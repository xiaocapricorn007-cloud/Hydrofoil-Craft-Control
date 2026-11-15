// FoilDynamics: Gazebo (gz-sim) system that applies the libfoil hydrofoil
// force model to a single rigid body.
//
// Gazebo integrates the rigid-body motion and applies gravity; this plugin
// supplies the hydrodynamic + propeller (+ air) wrench, the actuator lag and
// the unsteady-lift states.  With the phase 4 added mass switched on, the
// wrench is transformed so that Gazebo's plain rigid body (link inertia must be
// 215 kg, Ixx 31, Iyy 503) gets the accelerations of the body WITH added mass.  It also owns the wave field: the physics,
// the sonar sensor and the drawn water surface all sample the same object.
//
// Frames: Gazebo world is ENU-like (z up) and the link frame is FLU
// (x fwd, y left, z up).  libfoil uses NED / FRD.  Both pairs are related by a
// 180 deg rotation about x, i.e. vectors convert by flipping y and z:
//   N = x_gz, E = -y_gz, D = -z_gz ;  roll same, pitch and yaw change sign.
// The link origin MUST coincide with the centre of mass.
//
// gz topics
//   in : /hydrofoil/cmd/foil_fs, foil_fp, foil_a  (gz.msgs.Double, rad, absolute)
//        /hydrofoil/cmd/rpm                       (gz.msgs.Double)
//        /hydrofoil/env/params  (gz.msgs.Float_V)
//             [rho, aero(0/1), rho_air, wind_n, wind_e, cur_n, cur_e, physics mask]
//             (mask optional; < 0 = keep; bits = hf::Physics, 0 = reference, 127 = v2)
//        /hydrofoil/env/waves   (gz.msgs.Float_V)
//             [id, ramp_s, (amp, omega, psi, phase) x N]   new id => new sea
//   out: /hydrofoil/odom   (gz.msgs.Odometry: world pose, body-frame FLU twist)
//        /hydrofoil/debug  (gz.msgs.Float_V, layout in Publish())
//        /marker, /sensors/marker  (wave surface, gz.msgs.Marker)
//
// Time base of the waves: seconds since the craft was released from the rail.
//
// "Hold" mode: until the first command arrives the craft is carried on a rail
// at <initial_speed> and constant height, so that the controller starts from a
// clean trim state.  The foils sit at the trim of the active physics, solved
// by libfoil (<initial_delta> / <initial_rpm> are only the starting guess).
//
// <physics> SDF param: ref | v2 | mask | switch names (see hf::Physics).
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/AngularVelocityReset.hh>
#include <gz/sim/components/LinearVelocityReset.hh>
#include <gz/transport/Node.hh>

#include "hydrofoil/foil_model.hpp"

namespace hydrofoil
{

class FoilDynamics
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  ~FoilDynamics() override
  {
    {
      std::lock_guard<std::mutex> lk(this->visMtx);
      this->visStop = true;
    }
    this->visCv.notify_all();
    if (this->visThread.joinable()) {this->visThread.join();}
  }

  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    const std::string linkName = _sdf->Get<std::string>("link_name", "hull").first;
    this->link = gz::sim::Link(model.LinkByName(_ecm, linkName));
    if (!this->link.Valid(_ecm)) {
      gzerr << "[FoilDynamics] link '" << linkName << "' not found\n";
      return;
    }
    this->link.EnableVelocityChecks(_ecm, true);

    hf::Params P;
    P.rho = _sdf->Get<double>("rho", P.rho).first;
    P.wagner = _sdf->Get<bool>("wagner", true).first;
    P.act_tau = _sdf->Get<double>("actuator_tau", P.act_tau).first;
    // HYDROFOIL_PHYSICS (set by the launch file's physics:= argument) overrides the
    // SDF value, so the requested physics is active before the craft is released
    std::string phys = _sdf->Get<std::string>("physics", "ref").first;
    if (const char * ev = std::getenv("HYDROFOIL_PHYSICS"); ev && *ev) {phys = ev;}
    if (!hf::Physics::parse(phys, P.phys)) {
      gzerr << "[FoilDynamics] unknown <physics> '" << phys << "', using ref\n";
      P.phys = hf::Physics();
    }
    P.computeAddedMass();
    this->craft = hf::Craft(P);
    this->g = P.g;

    this->U0 = _sdf->Get<double>("initial_speed", 7.0).first;
    const auto d0 = _sdf->Get<gz::math::Vector3d>(
      "initial_delta", gz::math::Vector3d(0.05743, 0.05743, 0.05209)).first;
    this->cmd = {d0.X(), d0.Y(), d0.Z()};
    this->trim = this->cmd;
    this->rpm = _sdf->Get<double>("initial_rpm", 1170.4).first;
    this->lockLateral = _sdf->Get<bool>("lock_sway_yaw", true).first;
    this->hold = _sdf->Get<bool>("hold_until_command", true).first;
    this->Izz = _sdf->Get<double>("izz", 503.0).first;
    const double rate = _sdf->Get<double>("state_rate", 100.0).first;
    this->pubPeriod = Dur(1.0 / rate);
    const double vrate = _sdf->Get<double>("wave_visual_rate", 20.0).first;
    this->visPeriod = vrate > 0 ? Dur(1.0 / vrate) : Dur(0);
    this->visCell = _sdf->Get<double>("wave_visual_cell", 0.6).first;
    // water look (visual only)
    this->waterColor = _sdf->Get<gz::math::Vector3d>(
      "water_color", gz::math::Vector3d(0.05, 0.30, 0.52)).first;
    this->waterAlpha = _sdf->Get<double>("water_alpha", 1.0).first;
    this->waterRough = _sdf->Get<double>("water_roughness", -1.0).first;   // < 0: no PBR
    this->waterMetal = _sdf->Get<double>("water_metalness", 0.0).first;
    this->waterSpec = _sdf->Get<double>("water_specular", 0.5).first;
    this->visList = _sdf->Get<bool>("wave_visual_triangle_list", true).first;
    this->waterEnv = _sdf->Get<std::string>("water_environment_map", "").first;
    this->visHalfX = _sdf->Get<double>("wave_visual_half_x", 24.0).first;
    this->visHalfY = _sdf->Get<double>("wave_visual_half_y", 13.2).first;
    this->sonarPos = _sdf->Get<gz::math::Vector3d>(
      "sonar_position", gz::math::Vector3d(1.6, 0.0, -0.1)).first;      // FRD, thesis Table A.1
    this->sonarSigma = _sdf->Get<double>("sonar_noise", 0.002).first;
    this->sonarPeriod = Dur(1.0 / _sdf->Get<double>("sonar_rate", 20.0).first);

    // optional fixed regular wave(s):  <wave>amp omega psi phase</wave>
    if (_sdf->HasElement("wave")) {
      hf::WaveField wf;
      auto e = _sdf->FindElement("wave");
      while (e) {
        std::istringstream ss(e->Get<std::string>());
        double a, w, psi, ph;
        if (ss >> a >> w >> psi >> ph) {wf.add(a, w, psi, ph);}
        e = e->GetNextElement("wave");
      }
      this->sea = hf::Sea(wf);
    }

    const std::string ns = _sdf->Get<std::string>("topic_ns", "/hydrofoil").first;
    this->node.Subscribe(ns + "/cmd/foil_fs", &FoilDynamics::OnFs, this);
    this->node.Subscribe(ns + "/cmd/foil_fp", &FoilDynamics::OnFp, this);
    this->node.Subscribe(ns + "/cmd/foil_a", &FoilDynamics::OnA, this);
    this->node.Subscribe(ns + "/cmd/rpm", &FoilDynamics::OnRpm, this);
    this->node.Subscribe(ns + "/env/params", &FoilDynamics::OnEnv, this);
    this->node.Subscribe(ns + "/env/waves", &FoilDynamics::OnWaves, this);
    this->odomPub = this->node.Advertise<gz::msgs::Odometry>(ns + "/odom");
    this->dbgPub = this->node.Advertise<gz::msgs::Float_V>(ns + "/debug");
    this->configured = true;
    // the water surface is built and sent from its own thread: serialising tens of
    // thousands of marker points must not stall the physics step
    this->visThread = std::thread([this] {this->SeaWorker();});
    gzmsg << "[FoilDynamics] ready: physics=" << P.phys.str() << " rho=" << P.rho << " U0=" << this->U0
          << " hold=" << this->hold << " lock_sway_yaw=" << this->lockLateral << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override
  {
    if (!this->configured || _info.paused) {return;}
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double t = std::chrono::duration<double>(_info.simTime).count();

    const auto pose = this->link.WorldPose(_ecm);
    const auto vW = this->link.WorldLinearVelocity(_ecm);
    const auto wW = this->link.WorldAngularVelocity(_ecm);
    if (!pose || !vW || !wW) {return;}

    std::array<double, 3> c;
    double rpmNow;
    bool gotCmd;
    hf::Physics physNew;
    bool physChange = false;
    const double tw = this->releaseTime < 0 ? 0.0 : t - this->releaseTime;   // wave clock
    {
      std::lock_guard<std::mutex> lk(this->mtx);
      c = this->cmd; rpmNow = this->rpm; gotCmd = this->gotCmd;
      if (this->envPending) {this->craft.env() = this->envIn; this->envPending = false;}
      if (this->physPending) {physNew = this->physIn; this->physPending = false; physChange = true;}
      if (this->wavesPending) {
        this->sea.set(this->wavesIn, tw, this->wavesRamp);
        this->wavesPending = false;
        gzmsg << "[FoilDynamics] new sea: " << this->wavesIn.size() << " components, ramp "
              << this->wavesRamp << " s at wave time " << tw << "\n";
      }
    }

    const double mass = this->craft.params().mass;
    const double g = this->craft.params().g;

    // ---------------------------------------------------------------- hold
    if (!this->initialised || (this->hold && !gotCmd)) {
      // carry the craft on a rail: fixed velocity, gravity cancelled
      SetReset<gz::sim::components::WorldLinearVelocityReset>(
        _ecm, gz::math::Vector3d(this->U0, 0, 0));
      SetReset<gz::sim::components::WorldAngularVelocityReset>(
        _ecm, gz::math::Vector3d::Zero);
      this->link.AddWorldForce(_ecm, gz::math::Vector3d(0, 0, mass * g));
      // the wave field is anchored where the craft is released: wave x = 0 there
      this->originN = pose->Pos().X();
      this->originE = -pose->Pos().Y();
      hf::State s = this->ToState(*pose, gz::math::Vector3d(this->U0, 0, 0), gz::math::Vector3d::Zero);
      if (physChange && physNew.mask() != this->craft.params().phys.mask()) {
        this->craft.setPhysics(physNew, s);
        gzmsg << "[FoilDynamics] physics -> " << physNew.str() << " (on the rail)\n";
      }
      this->UpdateTrim(s);
      this->craft.reset(s, this->trim);
      this->initialised = true;
      this->releaseTime = -1.0;
      this->Sonar(_info, s, tw);
      this->Publish(_info, *pose, gz::math::Vector3d(this->U0, 0, 0), gz::math::Vector3d::Zero,
        rpmNow, c, tw, s);
      this->DrawSea(_info, s, tw);
      return;
    }
    if (this->releaseTime < 0) {
      this->releaseTime = t;
      gzmsg << "[FoilDynamics] released at t=" << t << " s\n";
    }

    // ------------------------------------------------------------- dynamics
    const auto q = pose->Rot();
    const gz::math::Vector3d vB = q.RotateVectorReverse(*vW);   // FLU
    const gz::math::Vector3d wB = q.RotateVectorReverse(*wW);
    hf::State s = this->ToState(*pose, *vW, *wW);
    if (physChange && physNew.mask() != this->craft.params().phys.mask()) {
      this->craft.setPhysics(physNew, s);
      gzmsg << "[FoilDynamics] physics -> " << physNew.str() << " at wave time " << tw << " s\n";
    }
    hf::Wrench W = this->craft.step(s, c, rpmNow, tw, dt, this->sea);

    if (this->lockLateral) {
      // thesis Sec. 3.1: sway, yaw rate and heading held at zero.  Implemented
      // as a stiff virtual damper/spring (the original model deletes the DOF).
      W.Y = -50.0 * mass * s.v;
      W.N = -(40.0 * s.r + 400.0 * s.psi) * this->Izz;
    }
    // FRD -> FLU -> world
    const gz::math::Vector3d fB(W.X, -W.Y, -W.Z);
    const gz::math::Vector3d tB(W.K, -W.M, -W.N);
    this->link.AddWorldWrench(_ecm, q.RotateVector(fB), q.RotateVector(tB));
    this->Sonar(_info, s, tw);
    this->Publish(_info, *pose, vB, wB, rpmNow, c, tw, s, true);
    this->DrawSea(_info, s, tw);
  }

private:
  using Duration = std::chrono::steady_clock::duration;
  static Duration Dur(double sec)
  {
    return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(sec));
  }

  // trim of the active physics at the rail speed and height (once per change)
  void UpdateTrim(const hf::State & s)
  {
    const int key = this->craft.params().phys.mask();
    if (key == this->trimKey && std::abs(s.z - this->trimZ) < 1e-3) {return;}
    hf::Craft tmp(this->craft.params());      // default (reference) environment
    std::array<double, 3> d = this->trim;
    double r = this->trimRpm > 0 ? this->trimRpm : this->rpm;
    if (tmp.trim(this->U0, s.z, d, r)) {
      this->trim = d;
      this->trimRpm = r;
      std::lock_guard<std::mutex> lk(this->mtx);
      if (!this->gotCmd) {this->cmd = d; this->rpm = r;}
      gzmsg << "[FoilDynamics] trim (" << this->craft.params().phys.str() << ", U " << this->U0
            << ", CG " << -s.z << " m above water): fore " << d[0] * 180 / M_PI << " aft "
            << d[2] * 180 / M_PI << " deg, " << r << " rpm\n";
    } else {
      gzwarn << "[FoilDynamics] trim failed; keeping the previous foil angles\n";
    }
    this->trimKey = key;
    this->trimZ = s.z;
  }

  template<typename C>
  void SetReset(gz::sim::EntityComponentManager & _ecm, const gz::math::Vector3d & v)
  {
    _ecm.SetComponentData<C>(this->link.Entity(), v);
  }

  hf::State ToState(
    const gz::math::Pose3d & pose, const gz::math::Vector3d & vW,
    const gz::math::Vector3d & wW) const
  {
    const auto q = pose.Rot();
    const gz::math::Vector3d vB = q.RotateVectorReverse(vW);
    const gz::math::Vector3d wB = q.RotateVectorReverse(wW);
    const gz::math::Vector3d e = q.Euler();   // roll, pitch, yaw (FLU / z-up)
    hf::State s;
    s.xN = pose.Pos().X() - this->originN;      // position relative to the release point
    s.yE = -pose.Pos().Y() - this->originE;
    s.z = -pose.Pos().Z();
    s.phi = e.X(); s.theta = -e.Y(); s.psi = -e.Z();
    s.u = vB.X(); s.v = -vB.Y(); s.w = -vB.Z();
    s.p = wB.X(); s.q = -wB.Y(); s.r = -wB.Z();
    return s;
  }

  double Zeta(double xN, double yE, double tw) const
  {
    double z; hf::Vec3 v;
    this->sea.sample(xN, yE, 0.0, tw, this->craft.params().g, z, v);
    return z;
  }

  // Downward-looking sonar fixed to the hull: range along the body z axis from
  // the sensor to the instantaneous wave surface, plus white noise, at 20 Hz.
  void Sonar(const gz::sim::UpdateInfo & _info, const hf::State & s, double tw)
  {
    if (_info.simTime < this->nextSonar) {return;}
    this->nextSonar = _info.simTime + this->sonarPeriod;
    double R[3][3];
    hf::dcm(s.phi, s.theta, s.psi, R);
    const auto & p = this->sonarPos;
    const double sx = s.xN + R[0][0] * p.X() + R[0][1] * p.Y() + R[0][2] * p.Z();
    const double sy = s.yE + R[1][0] * p.X() + R[1][1] * p.Y() + R[1][2] * p.Z();
    const double sz = s.z + R[2][0] * p.X() + R[2][1] * p.Y() + R[2][2] * p.Z();
    // ray: sensor + r * (body z axis in NED); surface is at z_ned = -zeta(x, y)
    double r = -sz / std::max(R[2][2], 0.2);
    for (int i = 0; i < 4; ++i) {
      const double zeta = this->Zeta(sx + r * R[0][2], sy + r * R[1][2], tw);
      r = (-zeta - sz) / std::max(R[2][2], 0.2);
    }
    this->zetaCg = this->Zeta(s.xN, s.yE, tw);
    this->sonarRange = r + this->sonarSigma * this->gauss(this->rng);
  }

  // vBody / wBody are body-frame (FLU) when `body` is true, world-frame otherwise
  void Publish(
    const gz::sim::UpdateInfo & _info, const gz::math::Pose3d & pose,
    const gz::math::Vector3d & v, const gz::math::Vector3d & w, double rpmNow,
    const std::array<double, 3> & c, double tw, const hf::State &, bool body = false)
  {
    if (_info.simTime < this->nextPub) {return;}
    this->nextPub += this->pubPeriod;
    if (this->nextPub < _info.simTime) {this->nextPub = _info.simTime + this->pubPeriod;}
    const auto q = pose.Rot();
    const gz::math::Vector3d vB = body ? v : q.RotateVectorReverse(v);
    const gz::math::Vector3d wB = body ? w : q.RotateVectorReverse(w);
    const auto sec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
    const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime - sec);

    gz::msgs::Odometry od;
    od.mutable_header()->mutable_stamp()->set_sec(sec.count());
    od.mutable_header()->mutable_stamp()->set_nsec(static_cast<int>(nsec.count()));
    od.mutable_pose()->mutable_position()->set_x(pose.Pos().X());
    od.mutable_pose()->mutable_position()->set_y(pose.Pos().Y());
    od.mutable_pose()->mutable_position()->set_z(pose.Pos().Z());
    od.mutable_pose()->mutable_orientation()->set_w(q.W());
    od.mutable_pose()->mutable_orientation()->set_x(q.X());
    od.mutable_pose()->mutable_orientation()->set_y(q.Y());
    od.mutable_pose()->mutable_orientation()->set_z(q.Z());
    od.mutable_twist()->mutable_linear()->set_x(vB.X());
    od.mutable_twist()->mutable_linear()->set_y(vB.Y());
    od.mutable_twist()->mutable_linear()->set_z(vB.Z());
    od.mutable_twist()->mutable_angular()->set_x(wB.X());
    od.mutable_twist()->mutable_angular()->set_y(wB.Y());
    od.mutable_twist()->mutable_angular()->set_z(wB.Z());
    this->odomPub.Publish(od);

    // debug vector layout (float32):
    //  0 release_time (-1 while held) | 1..3 actuator pos FS,FP,A [rad]
    //  4..6 submergence h FS,FP,A [m] | 7..9 alpha_eff [rad] | 10..12 lift [N]
    //  13 thrust [N] | 14 rpm | 15..17 commanded foil angles [rad] | 18 sim time
    //  19 sonar range [m] | 20 wave elevation at the CG [m] | 21 water density
    //  22 wave time (s since release) | 23,24 wind N,E | 25,26 current N,E
    //  27 physics mask | 28 wake upwash at the aft foil [m/s]
    gz::msgs::Float_V dv;
    const auto & a = this->craft.actuators();
    const auto & d = this->craft.debug();
    const auto & E = this->craft.env();
    auto add = [&dv](double x) {dv.add_data(static_cast<float>(x));};
    add(this->releaseTime);
    for (int i = 0; i < 3; ++i) {add(a[i]);}
    for (int i = 0; i < 3; ++i) {add(d[i].h);}
    for (int i = 0; i < 3; ++i) {add(d[i].alpha_eff);}
    for (int i = 0; i < 3; ++i) {add(d[i].lift);}
    add(this->craft.thrust());
    add(rpmNow);
    for (int i = 0; i < 3; ++i) {add(c[i]);}
    add(std::chrono::duration<double>(_info.simTime).count());
    add(this->sonarRange);
    add(this->zetaCg);
    add(E.rho > 0 ? E.rho : this->craft.params().rho);
    add(tw);
    add(E.wind_n); add(E.wind_e); add(E.cur_n); add(E.cur_e);
    add(this->craft.params().phys.mask());
    add(this->craft.upwash());
    this->dbgPub.Publish(dv);
  }

  // Draw the water surface around the craft as a triangle mesh, sampled
  // from the same wave field the foils see.  The patch is fixed to a world
  // lattice (so vertices do not swim) and tapers to the flat sea at its edge.
  struct SeaJob
  {
    hf::Sea sea;
    double xN, yE, tw, oN, oE;
  };

  // physics thread: hand the latest state to the drawing thread (newest job wins)
  void DrawSea(const gz::sim::UpdateInfo & _info, const hf::State & s, double tw)
  {
    if (this->visPeriod.count() == 0 || _info.simTime < this->nextVis) {return;}
    this->nextVis = _info.simTime + this->visPeriod;
    {
      std::lock_guard<std::mutex> lk(this->visMtx);
      this->visJob = SeaJob{this->sea, s.xN, s.yE, tw, this->originN, this->originE};
      this->visHave = true;
    }
    this->visCv.notify_one();
  }

  void SeaWorker()
  {
    while (true) {
      SeaJob job;
      {
        std::unique_lock<std::mutex> lk(this->visMtx);
        this->visCv.wait(lk, [this] {return this->visHave || this->visStop;});
        if (this->visStop) {return;}
        job = this->visJob;
        this->visHave = false;
      }
      this->BuildSea(job);
    }
  }

  void BuildSea(const SeaJob & job)
  {
    const auto wall0 = std::chrono::steady_clock::now();
    const double tw = job.tw;
    const double c = this->visCell;
    const int nx = static_cast<int>(2 * this->visHalfX / c) + 1;
    const int ny = static_cast<int>(2 * this->visHalfY / c) + 1;
    // lattice in Gazebo world coordinates (gz x = north, gz y = -east)
    const double x0 = std::round((job.xN + job.oN) / c) * c - this->visHalfX;
    const double y0 = std::round(-(job.yE + job.oE) / c) * c - this->visHalfY;
    const bool calm = job.sea.calm(tw);
    std::vector<float> z(static_cast<size_t>(nx) * ny, 0.0f);
    if (!calm) {
      // grid point (i, j): north = x0 + i c - originN, east = -(y0 + j c) - originE
      job.sea.elevationGrid(x0 - job.oN, -y0 - job.oE, c, -c, nx, ny, tw, this->g, z);
      for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
          const double fx = std::min(i, nx - 1 - i) / (0.18 * nx);
          const double fy = std::min(j, ny - 1 - j) / (0.18 * ny);
          z[static_cast<size_t>(j) * nx + i] *=
            static_cast<float>(std::min(1.0, std::min(fx, fy)));
        }
      }
    }
    gz::msgs::Marker m;
    m.set_ns("hydrofoil_sea");
    m.set_id(1);
    m.set_action(gz::msgs::Marker::ADD_MODIFY);
    m.set_type(this->visList ? gz::msgs::Marker::TRIANGLE_LIST : gz::msgs::Marker::TRIANGLE_STRIP);
    auto * mat = m.mutable_material();
    auto setc = [](gz::msgs::Color * col, float r, float g, float b, float a) {
        col->set_r(r); col->set_g(g); col->set_b(b); col->set_a(a);
      };
    const float wr = static_cast<float>(this->waterColor.X());
    const float wg = static_cast<float>(this->waterColor.Y());
    const float wb = static_cast<float>(this->waterColor.Z());
    const float wa = static_cast<float>(this->waterAlpha);
    const float ws = static_cast<float>(this->waterSpec);
    setc(mat->mutable_ambient(), 0.7f * wr, 0.7f * wg, 0.7f * wb, wa);
    setc(mat->mutable_diffuse(), wr, wg, wb, wa);
    setc(mat->mutable_specular(), ws, ws, ws, wa);
    if (this->waterRough >= 0.0) {
      auto * pbr = mat->mutable_pbr();
      pbr->set_type(gz::msgs::Material::PBR::METAL);
      pbr->set_roughness(this->waterRough);
      pbr->set_metalness(this->waterMetal);
      if (!this->waterEnv.empty()) {pbr->set_environment_map(this->waterEnv);}
    }
    auto pt = [&](int i, int j) {
        auto * p = m.add_point();
        p->set_x(x0 + i * c); p->set_y(y0 + j * c);
        p->set_z(z[static_cast<size_t>(j) * nx + i]);
      };
    if (this->visList) {
      // two counter-clockwise (seen from above) triangles per cell
      for (int j = 0; j < ny - 1; ++j) {
        for (int i = 0; i < nx - 1; ++i) {
          pt(i, j); pt(i + 1, j); pt(i + 1, j + 1);
          pt(i, j); pt(i + 1, j + 1); pt(i, j + 1);
        }
      }
    } else {
      for (int j = 0; j < ny - 1; ++j) {
        if (j > 0) {pt(0, j + 1);}                 // degenerate join between rows
        for (int i = 0; i < nx; ++i) {pt(i, j + 1); pt(i, j);}
        if (j < ny - 2) {pt(nx - 1, j);}
      }
    }
    const auto wall1 = std::chrono::steady_clock::now();
    this->node.Request("/marker", m);            // GUI scene
    this->node.Request("/sensors/marker", m);    // server-side camera sensors
    const auto wall2 = std::chrono::steady_clock::now();
    this->visBuildMs += std::chrono::duration<double, std::milli>(wall1 - wall0).count();
    this->visSendMs += std::chrono::duration<double, std::milli>(wall2 - wall1).count();
    if (++this->visCount == 200) {
      gzdbg << "[FoilDynamics] sea visual: " << m.point_size() << " points, build "
            << this->visBuildMs / 200 << " ms, send " << this->visSendMs / 200 << " ms per update\n";
      this->visCount = 0; this->visBuildMs = this->visSendMs = 0.0;
    }
  }

  void Set(int i, double v)
  {
    std::lock_guard<std::mutex> lk(this->mtx);
    if (std::isfinite(v)) {this->cmd[i] = v;}
    this->gotCmd = true;
  }
  void OnFs(const gz::msgs::Double & m) {Set(0, m.data());}
  void OnFp(const gz::msgs::Double & m) {Set(1, m.data());}
  void OnA(const gz::msgs::Double & m) {Set(2, m.data());}
  void OnRpm(const gz::msgs::Double & m)
  {
    std::lock_guard<std::mutex> lk(this->mtx);
    if (std::isfinite(m.data())) {this->rpm = m.data();}
  }
  void OnEnv(const gz::msgs::Float_V & m)
  {
    if (m.data_size() < 7) {return;}
    std::lock_guard<std::mutex> lk(this->mtx);
    this->envIn.rho = m.data(0);
    this->envIn.aero = m.data(1) > 0.5;
    this->envIn.rho_air = m.data(2);
    this->envIn.wind_n = m.data(3); this->envIn.wind_e = m.data(4);
    this->envIn.cur_n = m.data(5); this->envIn.cur_e = m.data(6);
    this->envPending = true;
    if (m.data_size() >= 8 && m.data(7) >= 0) {
      const int mask = static_cast<int>(std::lround(m.data(7)));
      if (mask != this->physMaskIn) {
        this->physMaskIn = mask;
        this->physIn = hf::Physics::fromMask(mask);
        this->physPending = true;
      }
    }
  }
  void OnWaves(const gz::msgs::Float_V & m)
  {
    if (m.data_size() < 2) {return;}
    std::lock_guard<std::mutex> lk(this->mtx);
    const int id = static_cast<int>(std::lround(m.data(0)));
    if (id == this->wavesId) {return;}           // periodic re-send of the same sea
    this->wavesId = id;
    this->wavesRamp = m.data(1);
    this->wavesIn.clear();
    for (int i = 2; i + 3 < m.data_size(); i += 4) {
      this->wavesIn.add(m.data(i), m.data(i + 1), m.data(i + 2), m.data(i + 3));
    }
    this->wavesPending = true;
  }

  std::thread visThread;
  std::mutex visMtx;
  std::condition_variable visCv;
  SeaJob visJob;
  bool visHave{false}, visStop{false};
  double g{9.81};
  gz::sim::Link link;
  hf::Craft craft;
  hf::Sea sea;
  gz::transport::Node node;
  gz::transport::Node::Publisher odomPub, dbgPub;
  std::mutex mtx;
  std::array<double, 3> cmd{}, trim{};
  hf::Env envIn;
  hf::WaveField wavesIn;
  double wavesRamp{5.0};
  int wavesId{-1};
  bool envPending{false}, wavesPending{false}, physPending{false};
  hf::Physics physIn;
  int physMaskIn{-1};
  int trimKey{-1};
  double trimZ{1e9}, trimRpm{-1.0};
  double rpm{0}, U0{7.0}, Izz{503.0};
  double originN{0.0}, originE{0.0};
  double releaseTime{-1.0};
  bool gotCmd{false}, hold{true}, lockLateral{true}, configured{false}, initialised{false};
  Duration pubPeriod{}, nextPub{}, visPeriod{}, nextVis{}, sonarPeriod{}, nextSonar{};
  double visCell{0.6}, visHalfX{24.0}, visHalfY{13.2};
  gz::math::Vector3d waterColor{0.05, 0.30, 0.52};
  double waterAlpha{1.0}, waterRough{-1.0}, waterMetal{0.0}, waterSpec{0.5};
  bool visList{true};
  int visCount{0};
  double visBuildMs{0.0}, visSendMs{0.0};
  std::string waterEnv;
  gz::math::Vector3d sonarPos;
  double sonarSigma{0.002}, sonarRange{0.0}, zetaCg{0.0};
  std::mt19937 rng{1234};
  std::normal_distribution<double> gauss{0.0, 1.0};
};

}  // namespace hydrofoil

GZ_ADD_PLUGIN(
  hydrofoil::FoilDynamics, gz::sim::System,
  hydrofoil::FoilDynamics::ISystemConfigure, hydrofoil::FoilDynamics::ISystemPreUpdate)
