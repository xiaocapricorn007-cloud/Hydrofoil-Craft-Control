// libfoil: hydrofoil craft force model, C++ port of hydrofoil/plant.py
// (itself a block-by-block port of jskoun/MPC_HydrofoilControl's Simulink model),
// plus the phase 3 environment (sea states that can change while running,
// current, wind drag, water density) mirroring hydrofoil/environment.py, and
// the phase 4 physics upgrades (Physics switches, mirror of params.Physics).
//
// Pure C++ (no Gazebo / ROS dependency) so it can be unit-tested against golden
// values exported from the validated Python model.
//
// Conventions: world NED (z down), body FRD (x fwd, y stbd, z down).
// The model returns the hydrodynamic + propeller (+ air) wrench about the CG
// in body axes.  Gravity is NOT included (the simulator applies it).
#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace hf
{

struct Vec3
{
  double x{0}, y{0}, z{0};
};

/// Linear deep-water wave field (thesis eqs. 2.4.8-2.4.12), sum of regular components.
/// psi = direction the waves travel TOWARDS (0 = +x_N).  zeta positive up.
class WaveField
{
public:
  struct Component
  {
    double amp, omega, psi, phase;
  };
  void clear() {comps_.clear();}
  void add(double amp, double omega, double psi, double phase)
  {
    comps_.push_back({amp, omega, psi, phase});
  }
  bool empty() const {return comps_.empty();}
  std::size_t size() const {return comps_.size();}
  const std::vector<Component> & components() const {return comps_;}
  /// elevation (up) and orbital velocity (NED) at a point `depth` below the mean surface
  void sample(double xN, double yE, double depth, double t, double g, double & zeta, Vec3 & vel) const;

private:
  std::vector<Component> comps_;
};

/// A sea that can be changed while running: each new wave field fades in over
/// `ramp` seconds while the previous one fades out (mirror of waves.SeaState).
class Sea
{
public:
  Sea() = default;
  /// a constant sea made of one field (full weight at all times)
  explicit Sea(const WaveField & f) {if (!f.empty()) {sets_.push_back({f, -1e9, 1.0, 0.0, 1.0, false});}}
  void set(const WaveField & f, double t, double ramp = 5.0);
  void sample(double xN, double yE, double depth, double t, double g, double & zeta, Vec3 & vel) const;
  bool calm(double t) const;
  /// Surface elevation on a regular grid, point (i, j) at (x0 + i*dx, y0 + j*dy) in
  /// NED north/east; out[j*nx + i].  Uses angle-addition so the cost is
  /// 2 multiplications per point per component (no trig in the inner loop).
  void elevationGrid(
    double x0, double y0, double dx, double dy, int nx, int ny, double t, double g,
    std::vector<float> & out) const;

private:
  struct Entry
  {
    WaveField field;
    double t_in, ramp_in, t_out, ramp_out;
    bool fading;
  };
  static double weight(const Entry & e, double t);
  std::vector<Entry> sets_;
};

struct FoilGeom
{
  Vec3 arm;          // position of the foil centre of pressure from CG, body FRD [m]
  double area;       // planform area [m^2]
  double strut_chord;
  double roll_int;   // int y^2 c(y) dy used by the roll-damping term
};

/// Phase 4 physics upgrades (mirror of hydrofoil/params.py Physics).  All off =
/// the reference model.  Bit order of mask(): the declaration order below.
struct Physics
{
  bool added_mass{false};      // foil + strut added mass in heave / roll / pitch
  bool speed_scaling{false};   // unsteady-lift poles and wave-drag Froude number use the actual speed
  bool lift_lag_only{false};   // lag only circulatory lift (Wagner for motion, Kussner for waves)
  bool roll_inflow{false};     // roll rate in the foil inflow; strip-theory roll damping
  bool ventilation{false};     // lift collapse near the surface, with hysteresis (ASSUMED numbers)
  bool downwash{false};        // fore-foil wake induces flow at the aft foil
  bool actuator_rate{false};   // servo rate limit
  static Physics v2() {return fromMask(0x7f);}
  static Physics fromMask(int m);
  int mask() const;
  bool any() const {return mask() != 0;}
  /// "ref", "v2", a mask number, or comma-separated switch names; false if not understood
  static bool parse(const std::string & text, Physics & out);
  std::string str() const;
};

struct Params
{
  Physics phys;
  double mass{215.0};
  double g{9.81};
  double rho{1024.8};
  std::array<FoilGeom, 3> foils;   // FS, FP, A
  double span{0.76};
  double mac{0.08};
  double t_foil{0.12 * 0.08};
  double prop_D{0.3};
  double prop_wake{0.05};
  double prop_arm_z{0.96};
  double usted{7.0};
  double wagC1{0.165}, wagC2{0.335};
  double wag_e1, wag_e2;
  double act_tau{0.1};
  double max_deflect{10.0 * M_PI / 180.0};
  bool wagner{true};
  // air drag (ASSUMED values, see hydrofoil/params.py)
  double air_CdA_x{0.5}, air_CdA_y{1.4}, air_z_ce{0.35};
  double Ixx{31.0}, Iyy{503.0};
  // phase 4 (ASSUMED values marked in hydrofoil/params.py)
  double vent_h_on{0.5}, vent_h_off{1.5};      // chords
  double vent_lift_factor{0.3};
  double vent_tau_on{0.03}, vent_tau_off{0.25};
  double act_rate{60.0 * M_PI / 180.0};        // rad/s
  double strut_cla{2 * M_PI};
  // added mass (computeAddedMass(); depends on rho): per-foil A33 and MA over [w, p, q]
  std::array<double, 3> A33{};
  double MA[3][3]{};
  Params();   // nominal values (params.py)
  void computeAddedMass();
};

/// Internal filter states of the phase 4 model (mirror of plant.Unsteady).
struct Unsteady
{
  std::array<std::array<double, 2>, 3> wag{};   // Wagner
  std::array<std::array<double, 2>, 3> kus{};   // Kussner
  std::array<double, 3> vent{};                 // ventilated fraction 0..1
  std::array<double, 2> gam{};                  // lagged fore-foil circulation [m^2/s]
};

/// Environment seen by the force model (mirror of plant.Env).
struct Env
{
  double rho{-1.0};                  // water density; < 0 = Params.rho
  double cur_n{0}, cur_e{0};         // water velocity over ground, NED [m/s]
  double wind_n{0}, wind_e{0};       // air velocity over ground, NED [m/s]
  bool aero{false};                  // apply air drag (off = reference model)
  double rho_air{1.225};
};

/// Rigid-body state needed by the force model.
struct State
{
  double xN{0}, yE{0}, z{0};           // CG position, NED
  double phi{0}, theta{0}, psi{0};     // Euler zyx
  double u{0}, v{0}, w{0};             // body velocities
  double p{0}, q{0}, r{0};             // body rates
};

struct Wrench
{
  double X{0}, Y{0}, Z{0}, K{0}, M{0}, N{0};
};

struct FoilDebug
{
  double h{0};          // submergence [m]
  double alpha{0};      // quasi-steady hydrodynamic angle [rad]
  double alpha_eff{0};  // after Wagner [rad]
  double lift{0}, drag{0};
  bool broached{false};
};

/// Time derivatives of the actuator and filter states plus the wrench (mirror of
/// plant.Hydrofoil.deriv).  `pole` holds, per filter state, the rate k of its
/// first-order form x' = k (target - x), used for exact discrete updates.
struct Deriv
{
  Wrench W;                        // hydro + propeller (+ air) + added-mass reaction, no gravity
  std::array<double, 3> dact{};
  Unsteady dx;
  Unsteady pole;
};

class Craft
{
public:
  explicit Craft(const Params & p = Params());

  /// Put actuators at `delta` and all filters at steady state for `s`.
  void reset(const State & s, const std::array<double, 3> & delta);
  /// Filters to steady state at `s`, actuators kept (used when the physics change).
  void resetFilters(const State & s);

  /// Wrench for given foil angles.  ust == nullptr: quasi-steady (no filters,
  /// ventilation decided by depth alone, current circulation in the downwash).
  /// alpha_out receives the quasi-steady hydrodynamic angles.
  Wrench forcesU(
    const State & s, const std::array<double, 3> & delta, double rpm, double t,
    const Sea & sea, const Unsteady * ust, std::array<double, 3> * alpha_out = nullptr) const;
  /// Reference-model interface: use_wagner_states = use the internal filter states.
  Wrench forces(
    const State & s, const std::array<double, 3> & delta, double rpm, double t,
    const Sea & sea, bool use_wagner_states,
    std::array<double, 3> * alpha_out = nullptr) const
  {
    return forcesU(s, delta, rpm, t, sea, use_wagner_states ? &ust_ : nullptr, alpha_out);
  }

  /// State derivatives at explicit actuator / filter states.
  Deriv deriv(
    const State & s, const std::array<double, 3> & act, const Unsteady & ust,
    const std::array<double, 3> & delta_cmd, double rpm, double t, const Sea & sea) const;

  /// [w', p', q'] of the reduced model (plant.core_deriv) for wrench W (no gravity),
  /// including added mass when it is switched on.
  std::array<double, 3> accelWPQ(const State & s, const Wrench & W) const;
  /// With added mass on: the wrench that gives a plain rigid body (mass, Ixx, Iyy,
  /// gravity applied by the simulator) the accelerations accelWPQ().  Otherwise W.
  Wrench applyAddedMass(const State & s, const Wrench & W) const;

  /// Trim (u' = w' = q' = 0, quasi-steady forces, calm water) at speed U and CG
  /// depth z (NED, negative = above the water): equal fore foils, aft foil, rpm.
  /// Newton iteration from the values passed in.  Returns false if it fails.
  bool trim(double U, double z, std::array<double, 3> & delta, double & rpm) const;

  /// One simulation step: wrench at the current internal states (to apply to a
  /// rigid body that has no added mass), then advance actuators and filters by dt.
  Wrench step(
    const State & s, const std::array<double, 3> & delta_cmd, double rpm, double t, double dt,
    const Sea & sea);

  /// Change the physics switches in flight (filters re-initialised at `s`).
  void setPhysics(const Physics & ph, const State & s);

  const std::array<double, 3> & actuators() const {return act_;}
  const Unsteady & filters() const {return ust_;}
  const std::array<FoilDebug, 3> & debug() const {return dbg_;}
  double thrust() const {return thrust_;}
  Params & params() {return P_;}
  const Params & params() const {return P_;}
  Env & env() {return E_;}
  const Env & env() const {return E_;}

  double upwash() const {return upwash_;}

private:
  struct Last
  {
    std::array<double, 3> a_mot{}, a_wav{}, V{}, h{}, alpha{};
    std::array<double, 2> gam{};
  };
  double wakeUpwash(double h) const;   // upwash at the aft foil per unit fore-foil circulation
  Params P_;
  Env E_;
  std::array<double, 3> act_{};
  Unsteady ust_;
  mutable std::array<FoilDebug, 3> dbg_{};
  mutable double thrust_{0};
  mutable double upwash_{0};
  mutable Last last_;
};

/// Linear interpolation with linear extrapolation (Simulink n-D lookup default).
double interp1(const double * xp, const double * fp, int n, double x);

/// R^n_b (zyx Euler), row-major.
void dcm(double phi, double theta, double psi, double R[3][3]);

}  // namespace hf
