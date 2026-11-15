#include "hydrofoil/foil_model.hpp"

#include <algorithm>
#include <cctype>
#include <string>

#include "hydrofoil/generated_data.hpp"

namespace hf
{

double interp1(const double * xp, const double * fp, int n, double x)
{
  if (x <= xp[0]) {
    return fp[0] + (fp[1] - fp[0]) / (xp[1] - xp[0]) * (x - xp[0]);
  }
  if (x >= xp[n - 1]) {
    return fp[n - 1] + (fp[n - 1] - fp[n - 2]) / (xp[n - 1] - xp[n - 2]) * (x - xp[n - 1]);
  }
  const double * it = std::upper_bound(xp, xp + n, x);
  int i = static_cast<int>(it - xp) - 1;
  double a = (x - xp[i]) / (xp[i + 1] - xp[i]);
  return fp[i] + a * (fp[i + 1] - fp[i]);
}

void WaveField::sample(
  double xN, double yE, double depth, double t, double g, double & zeta, Vec3 & vel) const
{
  zeta = 0.0;
  vel = Vec3{};
  const double d = std::max(depth, 0.0);
  for (const auto & c : comps_) {
    const double k = c.omega * c.omega / g;
    const double X = xN * std::cos(c.psi) + yE * std::sin(c.psi);
    const double ph = c.omega * t - k * X + c.phase;
    const double a = c.omega * c.amp * std::exp(-k * d);
    zeta += c.amp * std::sin(ph);
    vel.x += a * std::cos(c.psi) * std::sin(ph);
    vel.y += a * std::sin(c.psi) * std::sin(ph);
    vel.z += -a * std::cos(ph);
  }
}

namespace
{
double smooth(double x)
{
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3 - 2 * x);
}
}  // namespace

double Sea::weight(const Entry & e, double t)
{
  double w = smooth((t - e.t_in) / e.ramp_in);
  if (e.fading) {w *= 1.0 - smooth((t - e.t_out) / e.ramp_out);}
  return w;
}

void Sea::set(const WaveField & f, double t, double ramp)
{
  ramp = std::max(ramp, 1e-3);
  for (auto & e : sets_) {
    if (!e.fading) {e.fading = true; e.t_out = t; e.ramp_out = ramp;}
  }
  sets_.erase(
    std::remove_if(
      sets_.begin(), sets_.end(),
      [t](const Entry & e) {return e.fading && t > e.t_out + e.ramp_out;}),
    sets_.end());
  if (!f.empty()) {sets_.push_back({f, t, ramp, 0.0, ramp, false});}
}

void Sea::sample(
  double xN, double yE, double depth, double t, double g, double & zeta, Vec3 & vel) const
{
  zeta = 0.0;
  vel = Vec3{};
  for (const auto & e : sets_) {
    const double w = weight(e, t);
    if (w <= 0.0) {continue;}
    double z; Vec3 v;
    e.field.sample(xN, yE, depth, t, g, z, v);
    zeta += w * z;
    vel.x += w * v.x; vel.y += w * v.y; vel.z += w * v.z;
  }
}

void Sea::elevationGrid(
  double x0, double y0, double dx, double dy, int nx, int ny, double t, double g,
  std::vector<float> & out) const
{
  out.assign(static_cast<std::size_t>(nx) * ny, 0.0f);
  std::vector<double> sa(nx), ca(nx), sb(ny), cb(ny);
  for (const auto & e : sets_) {
    const double w = weight(e, t);
    if (w <= 0.0) {continue;}
    for (const auto & c : e.field.components()) {
      const double k = c.omega * c.omega / g;
      const double kx = k * std::cos(c.psi), ky = k * std::sin(c.psi);
      const double A = c.omega * t + c.phase - kx * x0;
      for (int i = 0; i < nx; ++i) {
        const double a = A - kx * dx * i;
        sa[i] = std::sin(a); ca[i] = std::cos(a);
      }
      for (int j = 0; j < ny; ++j) {
        const double b = -ky * (y0 + dy * j);
        sb[j] = std::sin(b); cb[j] = std::cos(b);
      }
      const double amp = w * c.amp;
      for (int j = 0; j < ny; ++j) {
        float * row = out.data() + static_cast<std::size_t>(j) * nx;
        const double sbj = amp * sb[j], cbj = amp * cb[j];
        for (int i = 0; i < nx; ++i) {
          row[i] += static_cast<float>(sa[i] * cbj + ca[i] * sbj);
        }
      }
    }
  }
}

bool Sea::calm(double t) const
{
  for (const auto & e : sets_) {
    if (weight(e, t) > 0.0) {return false;}
  }
  return true;
}

// ------------------------------------------------------------------ Physics
namespace
{
const char * const kPhysNames[7] = {
  "added_mass", "speed_scaling", "lift_lag_only", "roll_inflow", "ventilation", "downwash",
  "actuator_rate"};
constexpr double WAG_B1 = 0.0455, WAG_B2 = 0.3;   // Wagner (R.T. Jones) poles, semi-chords
constexpr double KUS_B1 = 0.13, KUS_B2 = 1.0;     // Kussner (Sears & Sparks) poles

// d fp / d xp of the piecewise-linear table at x (numpy searchsorted 'left' convention)
double slope(const double * xp, const double * fp, int n, double x)
{
  int i = static_cast<int>(std::lower_bound(xp, xp + n, x) - xp) - 1;
  i = std::clamp(i, 0, n - 2);
  return (fp[i + 1] - fp[i]) / (xp[i + 1] - xp[i]);
}

// np.interp: linear, clamped at the ends
double interpClamp(const double * xp, const double * fp, int n, double x)
{
  if (x <= xp[0]) {return fp[0];}
  if (x >= xp[n - 1]) {return fp[n - 1];}
  const double * it = std::upper_bound(xp, xp + n, x);
  const int i = static_cast<int>(it - xp) - 1;
  return fp[i] + (x - xp[i]) / (xp[i + 1] - xp[i]) * (fp[i + 1] - fp[i]);
}

bool solve3(const double A[3][3], const double b[3], double x[3])
{
  double M[3][4];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {M[i][j] = A[i][j];}
    M[i][3] = b[i];
  }
  for (int c = 0; c < 3; ++c) {
    int piv = c;
    for (int r = c + 1; r < 3; ++r) {
      if (std::abs(M[r][c]) > std::abs(M[piv][c])) {piv = r;}
    }
    if (std::abs(M[piv][c]) < 1e-300) {return false;}
    if (piv != c) {
      for (int j = 0; j < 4; ++j) {std::swap(M[c][j], M[piv][j]);}
    }
    for (int r = 0; r < 3; ++r) {
      if (r == c) {continue;}
      const double f = M[r][c] / M[c][c];
      for (int j = c; j < 4; ++j) {M[r][j] -= f * M[c][j];}
    }
  }
  for (int i = 0; i < 3; ++i) {x[i] = M[i][3] / M[i][i];}
  return true;
}
}  // namespace

Physics Physics::fromMask(int m)
{
  Physics p;
  p.added_mass = m & 1; p.speed_scaling = m & 2; p.lift_lag_only = m & 4;
  p.roll_inflow = m & 8; p.ventilation = m & 16; p.downwash = m & 32; p.actuator_rate = m & 64;
  return p;
}

int Physics::mask() const
{
  return (added_mass ? 1 : 0) | (speed_scaling ? 2 : 0) | (lift_lag_only ? 4 : 0) |
         (roll_inflow ? 8 : 0) | (ventilation ? 16 : 0) | (downwash ? 32 : 0) |
         (actuator_rate ? 64 : 0);
}

bool Physics::parse(const std::string & text, Physics & out)
{
  std::string t;
  for (char c : text) {
    if (!std::isspace(static_cast<unsigned char>(c))) {
      t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
  }
  if (t.empty() || t == "ref" || t == "reference" || t == "off") {out = Physics(); return true;}
  if (t == "v2" || t == "all" || t == "on") {out = v2(); return true;}
  if (std::isdigit(static_cast<unsigned char>(t[0]))) {
    int m = 0;
    for (char c : t) {
      if (!std::isdigit(static_cast<unsigned char>(c))) {return false;}
      m = m * 10 + (c - '0');
      if (m > 127) {return false;}
    }
    out = fromMask(m);
    return true;
  }
  int m = 0;
  std::size_t a = 0;
  while (a <= t.size()) {
    std::size_t b = t.find(',', a);
    if (b == std::string::npos) {b = t.size();}
    const std::string tok = t.substr(a, b - a);
    int k = -1;
    for (int i = 0; i < 7; ++i) {
      if (tok == kPhysNames[i]) {k = i;}
    }
    if (k < 0) {return false;}
    m |= 1 << k;
    a = b + 1;
  }
  out = fromMask(m);
  return true;
}

std::string Physics::str() const
{
  const int m = mask();
  if (m == 0) {return "ref";}
  if (m == 0x7f) {return "v2";}
  std::string s;
  for (int i = 0; i < 7; ++i) {
    if (m & (1 << i)) {s += (s.empty() ? "" : ",") + std::string(kPhysNames[i]);}
  }
  return s;
}

// ------------------------------------------------------------------- Params
Params::Params()
{
  foils[0] = {{1.1, 0.62, 0.75}, 0.048, 0.08, data::ROLL_INT_FS};    // FS
  foils[1] = {{1.1, -0.62, 0.75}, 0.048, 0.08, data::ROLL_INT_FP};   // FP
  foils[2] = {{-2.45, 0.0, 0.96}, 0.046, 0.10, data::ROLL_INT_A};    // A
  wag_e1 = 0.0455 * (usted / (mac / 2));
  wag_e2 = 0.3 * (usted / (mac / 2));
  computeAddedMass();
}

void Params::computeAddedMass()
{
  // mirror of params.Params._added_mass (strip theory, Scherer AR factor)
  for (auto & row : MA) {
    for (double & v : row) {v = 0.0;}
  }
  for (int i = 0; i < 3; ++i) {
    const FoilGeom & f = foils[i];
    const double ar = span * span / f.area;
    const double c0 = 4 * f.area / (M_PI * span);
    const double a33 = M_PI * rho * span * c0 * c0 / 6 * ar / std::sqrt(1 + ar * ar);
    A33[i] = a33;
    const double j[3] = {1.0, f.arm.y, -f.arm.x};
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {MA[a][b] += a33 * j[a] * j[b];}
    }
    MA[1][1] += M_PI * rho * c0 * c0 * span * span * span / 120;
    const double zwl = 0.3;
    const double lz = f.arm.z;
    MA[1][1] += M_PI * rho * f.strut_chord * f.strut_chord / 4 * (lz * lz * lz - zwl * zwl * zwl) / 3;
  }
}

Craft::Craft(const Params & p)
: P_(p) {}

void dcm(double phi, double th, double psi, double R[3][3])
{
  const double cf = std::cos(phi), sf = std::sin(phi);
  const double ct = std::cos(th), st = std::sin(th);
  const double cp = std::cos(psi), sp = std::sin(psi);
  R[0][0] = cp * ct; R[0][1] = -sp * cf + cp * st * sf; R[0][2] = sp * sf + cp * cf * st;
  R[1][0] = sp * ct; R[1][1] = cp * cf + sf * st * sp;  R[1][2] = -cp * sf + st * sp * cf;
  R[2][0] = -st;     R[2][1] = sf * ct;                 R[2][2] = cf * ct;
}

double Craft::wakeUpwash(double h) const
{
  return interpClamp(
    data::WAKE_H.data(), data::WAKE_G.data(), static_cast<int>(data::WAKE_H.size()), h);
}

// ------------------------------------------------------------------- forces
Wrench Craft::forcesU(
  const State & s, const std::array<double, 3> & delta, double rpm, double t,
  const Sea & sea, const Unsteady * ust, std::array<double, 3> * alpha_out) const
{
  const Params & P = P_;
  const Physics & F = P.phys;
  const Env & E = E_;
  const double rho = E.rho > 0 ? E.rho : P.rho;
  double R[3][3];
  dcm(s.phi, s.theta, s.psi, R);
  // current in body axes (R^T * [cur_n, cur_e, 0])
  const double cbx = R[0][0] * E.cur_n + R[1][0] * E.cur_e;
  const double cbz = R[0][2] * E.cur_n + R[1][2] * E.cur_e;
  const double u_w = s.u - cbx;               // forward speed through the water
  Wrench W;
  const int n = static_cast<int>(data::POLAR_ALPHA_DEG.size());
  const double * xa = data::POLAR_ALPHA_DEG.data();
  const double bprime = M_PI / 4 * P.span;
  Last lt;
  double w_up = 0.0;

  for (int i = 0; i < 3; ++i) {
    const FoilGeom & f = P.foils[i];
    const double lx = f.arm.x, ly = f.arm.y, lz = f.arm.z;
    // foil position in NED
    const double px = s.xN + R[0][0] * lx + R[0][1] * ly + R[0][2] * lz;
    const double py = s.yE + R[1][0] * lx + R[1][1] * ly + R[1][2] * lz;
    const double pz = s.z + R[2][0] * lx + R[2][1] * ly + R[2][2] * lz;
    double zeta; Vec3 vw;
    sea.sample(px, py, pz, t, P.g, zeta, vw);
    const double h = pz + zeta;
    vw.x += E.cur_n; vw.y += E.cur_e;         // water velocity = orbital + current
    // pivot velocity: v + omega x r.  Reference: ly omitted (as in the Simulink model)
    const double vpx = s.u + s.q * lz;
    double vpz = s.w - s.q * lx + (F.roll_inflow ? s.p * ly : 0.0);
    if (i == 2 && F.downwash) {
      const auto & g = ust ? ust->gam : lt.gam;
      w_up = wakeUpwash(lt.h[0]) * g[0] + wakeUpwash(lt.h[1]) * g[1];
      vpz += w_up;                            // upwash = larger incidence
    }
    // water velocity into body axes (R^T * vw)
    const double wbx = R[0][0] * vw.x + R[1][0] * vw.y + R[2][0] * vw.z;
    const double wbz = R[0][2] * vw.x + R[1][2] * vw.y + R[2][2] * vw.z;
    const double urel = vpx - wbx;
    const double wrel = vpz - wbz;
    const double a_inf = std::atan(wrel / urel);
    const double V2 = urel * urel + wrel * wrel;
    const double V = std::sqrt(V2);
    const double alpha = a_inf + delta[i];
    const double a_m = std::atan((vpz - cbz) / (vpx - cbx)) + delta[i];
    lt.a_mot[i] = a_m; lt.a_wav[i] = alpha - a_m; lt.V[i] = V; lt.h[i] = h; lt.alpha[i] = alpha;
    if (alpha_out) {(*alpha_out)[i] = alpha;}
    double a_circ, a_visc;
    if (!ust) {
      a_circ = a_visc = alpha;
    } else if (F.lift_lag_only) {
      a_circ = 0.5 * a_m + ust->wag[i][0] + ust->wag[i][1] + ust->kus[i][0] + ust->kus[i][1];
      a_visc = alpha;
    } else {
      a_circ = a_visc = P.wagner ? 0.5 * alpha + ust->wag[i][0] + ust->wag[i][1] : alpha;
    }
    const double ac_deg = a_circ * 180.0 / M_PI, av_deg = a_visc * 180.0 / M_PI;
    FoilDebug & D = dbg_[i];
    D.h = h; D.alpha = alpha; D.alpha_eff = a_circ; D.lift = 0; D.drag = 0;
    D.broached = h <= 0.0;
    if (F.roll_inflow) {
      // strip theory over the foil's own span with the local polar lift slope
      const double cla = slope(xa, data::POLAR_CL.data(), n, ac_deg) * 180.0 / M_PI;
      W.K += -0.5 * rho * cla * (f.area * P.span * P.span / 16) * V * s.p;
    } else {
      // reference: -1/2 rho 2pi Ld * u_pivot(through water) * p
      W.K += -0.5 * rho * 2 * M_PI * f.roll_int * (vpx - cbx) * s.p;
    }
    if (h <= 0.0) {continue;}   // broached: forces set to zero (documented deviation)

    double cl = interp1(xa, data::POLAR_CL.data(), n, ac_deg);
    double cdi = interp1(xa, data::POLAR_CDI.data(), n, ac_deg);
    const double cdv = interp1(xa, data::POLAR_CDV.data(), n, av_deg);
    double cm = interp1(xa, data::POLAR_CM.data(), n, av_deg);
    if (F.ventilation) {
      const double v = ust ? ust->vent[i] : (h < P.vent_h_on * P.mac ? 1.0 : 0.0);
      const double kv = 1.0 - (1.0 - P.vent_lift_factor) * v;
      cl *= kv; cdi *= kv * kv; cm *= kv;
    }
    const double ldec = 1 - 0.422 * std::exp(-1.454 * h / P.mac);
    const double x15 = 33 * std::pow(h / P.span, 1.5);
    const double dinc = (2 - x15 / (x15 + 1)) * cdi;
    const double frh = (F.speed_scaling ? V : P.usted) / std::sqrt(P.g * h);
    const double cdw = 1 / (2 * frh * frh) * (P.mac / h) * std::exp(-2 / (frh * frh));
    const double tc = P.t_foil / P.mac;
    const double cdint = 0.75 * tc - 0.0003 / (tc * tc);
    const double qS = 0.5 * rho * f.area * V2;
    const double L = qS * cl * ldec;
    const double Dg = qS * (cdv + dinc) + qS * cl * cl * cdw +
      0.5 * rho * P.t_foil * P.t_foil * V2 * cdint;
    const double Mf = qS * P.mac * cm * ldec;
    if (i < 2) {lt.gam[i] = L / (rho * V * bprime);}   // horseshoe circulation
    const double ca = std::cos(a_inf), sa = std::sin(a_inf);
    const double Fx = -ca * Dg + sa * L;
    const double Fz = -sa * Dg - ca * L;
    // moments r x F with F = [Fx, 0, Fz]
    const double Mx = ly * Fz;
    const double My = lz * Fx - lx * Fz;
    const double Mz = -ly * Fx;
    // strut: drag, its pitch moment, roll damping
    const double Xs = -0.5 * rho * f.strut_chord * u_w * u_w * data::CD_STRUT0 * h;
    const double Ms = Xs * (lz - 0.5 * h);
    double Ks;
    if (F.roll_inflow) {        // wetted strut from the waterline (lz - h) down to the foil
      const double zt = lz - h;
      Ks = -0.5 * rho * u_w * P.strut_cla * s.p * f.strut_chord * (lz * lz * lz - zt * zt * zt) / 3.0;
    } else {                    // reference formula, including its 0.1 m offset
      const double zwl = lz + 0.1 - h;
      Ks = -0.5 * rho * u_w * 2 * M_PI * s.p * f.strut_chord * (lz * lz * lz - zwl * zwl * zwl) / 3.0;
    }
    W.X += Fx + Xs;
    W.Z += Fz;
    W.K += Mx + Ks;
    W.M += My + Mf + Ms;
    W.N += Mz;
    D.lift = L; D.drag = Dg;
  }
  // propeller
  const double nrev = rpm / 60.0;
  double T = 0.0;
  if (std::abs(nrev) > 1e-6) {
    const double J = (1 - P.prop_wake) * u_w / (nrev * P.prop_D);
    const double kt = (data::KT_POLY[0] * J + data::KT_POLY[1]) * J + data::KT_POLY[2];
    T = rho * nrev * std::abs(nrev) * std::pow(P.prop_D, 4) * kt;
  }
  thrust_ = T;
  W.X += T;
  W.M += P.prop_arm_z * T;
  // air drag on hull / deck
  if (E.aero) {
    const double wbx = R[0][0] * E.wind_n + R[1][0] * E.wind_e;
    const double wby = R[0][1] * E.wind_n + R[1][1] * E.wind_e;
    const double vax = s.u - wbx, vay = -wby;
    const double Fax = -0.5 * E.rho_air * P.air_CdA_x * vax * std::abs(vax);
    const double Fay = -0.5 * E.rho_air * P.air_CdA_y * vay * std::abs(vay);
    W.X += Fax;
    W.Y += Fay;
    W.K += P.air_z_ce * Fay;
    W.M += -P.air_z_ce * Fax;
  }
  upwash_ = w_up;
  last_ = lt;
  return W;
}

// -------------------------------------------------------------- derivatives
Deriv Craft::deriv(
  const State & s, const std::array<double, 3> & act, const Unsteady & ust,
  const std::array<double, 3> & delta_cmd, double rpm, double t, const Sea & sea) const
{
  const Params & P = P_;
  const Physics & F = P.phys;
  Deriv D;
  std::array<double, 3> al{};
  D.W = forcesU(s, act, rpm, t, sea, &ust, &al);
  const Last lt = last_;
  for (int i = 0; i < 3; ++i) {
    D.dact[i] = (delta_cmd[i] - act[i]) / P.act_tau;
    if (F.actuator_rate) {D.dact[i] = std::clamp(D.dact[i], -P.act_rate, P.act_rate);}
  }
  if (F.added_mass) {
    // non-circulatory reaction of the foil rotating in the stream: Fz = -A33 U delta_dot
    for (int i = 0; i < 3; ++i) {
      const double Fz = -P.A33[i] * lt.V[i] * D.dact[i];
      D.W.Z += Fz;
      D.W.K += P.foils[i].arm.y * Fz;
      D.W.M += -P.foils[i].arm.x * Fz;
    }
  }
  const double hc = P.mac / 2;
  if (P.wagner) {
    for (int i = 0; i < 3; ++i) {
      const double Vs = F.speed_scaling ? lt.V[i] : P.usted;
      const double drive = F.lift_lag_only ? lt.a_mot[i] : al[i];
      const double k1 = WAG_B1 * Vs / hc, k2 = WAG_B2 * Vs / hc;
      D.pole.wag[i] = {k1, k2};
      D.dx.wag[i][0] = k1 * (P.wagC1 * drive - ust.wag[i][0]);
      D.dx.wag[i][1] = k2 * (P.wagC2 * drive - ust.wag[i][1]);
      if (F.lift_lag_only) {
        const double c1 = KUS_B1 * Vs / hc, c2 = KUS_B2 * Vs / hc;
        D.pole.kus[i] = {c1, c2};
        D.dx.kus[i][0] = c1 * (0.5 * lt.a_wav[i] - ust.kus[i][0]);
        D.dx.kus[i][1] = c2 * (0.5 * lt.a_wav[i] - ust.kus[i][1]);
      }
    }
  }
  if (F.ventilation) {
    for (int i = 0; i < 3; ++i) {
      const double v = ust.vent[i];
      if (lt.h[i] < P.vent_h_on * P.mac) {
        D.pole.vent[i] = 1 / P.vent_tau_on; D.dx.vent[i] = (1 - v) / P.vent_tau_on;
      } else if (lt.h[i] > P.vent_h_off * P.mac) {
        D.pole.vent[i] = 1 / P.vent_tau_off; D.dx.vent[i] = -v / P.vent_tau_off;
      }
    }
  }
  if (F.downwash) {
    // the wake needs (x_fore - x_aft) / V to reach the aft foil: first-order lag
    for (int i = 0; i < 2; ++i) {
      const double tw = (P.foils[0].arm.x - P.foils[2].arm.x) / std::max(lt.V[i], 1.0);
      D.pole.gam[i] = 1 / tw;
      D.dx.gam[i] = (lt.gam[i] - ust.gam[i]) / tw;
    }
  }
  return D;
}

std::array<double, 3> Craft::accelWPQ(const State & s, const Wrench & W) const
{
  const Params & P = P_;
  const double Z = W.Z + P.mass * P.g * std::cos(s.theta) * std::cos(s.phi);
  if (!P.phys.added_mass) {
    return {Z / P.mass + s.q * s.u, W.K / P.Ixx, W.M / P.Iyy};
  }
  double A[3][3];
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {A[a][b] = P.MA[a][b];}
  }
  A[0][0] += P.mass; A[1][1] += P.Ixx; A[2][2] += P.Iyy;
  const double rhs[3] = {Z + P.mass * s.q * s.u, W.K, W.M};
  double x[3] = {0, 0, 0};
  solve3(A, rhs, x);
  return {x[0], x[1], x[2]};
}

Wrench Craft::applyAddedMass(const State & s, const Wrench & W) const
{
  if (!P_.phys.added_mass) {return W;}
  const Params & P = P_;
  const auto a = accelWPQ(s, W);
  const double G = P.mass * P.g * std::cos(s.theta) * std::cos(s.phi);
  Wrench out = W;
  // plain rigid body: m (w' - q u) = Z + G,  Ixx p' = K,  Iyy q' = M  (v, r ~ 0)
  out.Z = P.mass * (a[0] - s.q * s.u) - G;
  out.K = P.Ixx * a[1];
  out.M = P.Iyy * a[2];
  return out;
}

// --------------------------------------------------------------------- trim
bool Craft::trim(double U, double z, std::array<double, 3> & delta, double & rpm) const
{
  State s;
  s.z = z; s.u = U;
  const Sea calm;
  auto res = [&](const double v[3], double r[3]) {
      const std::array<double, 3> d{v[0], v[0], v[1]};
      const Wrench W = forcesU(s, d, v[2], 0.0, calm, nullptr);
      const auto a = accelWPQ(s, W);
      r[0] = W.X / P_.mass;      // theta = 0: no gravity along x
      r[1] = a[0];
      r[2] = a[2];
    };
  double v[3] = {delta[0], delta[2], rpm};
  const double hstep[3] = {1e-6, 1e-6, 1e-3};
  for (int it = 0; it < 60; ++it) {
    double r[3];
    res(v, r);
    if (std::abs(r[0]) + std::abs(r[1]) + std::abs(r[2]) < 1e-11) {
      delta = {v[0], v[0], v[1]};
      rpm = v[2];
      return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    }
    double J[3][3];
    for (int j = 0; j < 3; ++j) {
      double vp[3] = {v[0], v[1], v[2]}, vm[3] = {v[0], v[1], v[2]};
      vp[j] += hstep[j]; vm[j] -= hstep[j];
      double rp[3], rm[3];
      res(vp, rp); res(vm, rm);
      for (int i = 0; i < 3; ++i) {J[i][j] = (rp[i] - rm[i]) / (2 * hstep[j]);}
    }
    double dv[3];
    if (!solve3(J, r, dv)) {return false;}
    for (int j = 0; j < 3; ++j) {v[j] -= dv[j];}
  }
  return false;
}

// --------------------------------------------------------------- simulation
void Craft::resetFilters(const State & s)
{
  std::array<double, 3> al{};
  const Sea calm;
  forcesU(s, act_, 0.0, 0.0, calm, nullptr, &al);
  const Last lt = last_;
  ust_ = Unsteady{};
  if (P_.wagner) {
    for (int i = 0; i < 3; ++i) {
      const double drive = P_.phys.lift_lag_only ? lt.a_mot[i] : al[i];
      ust_.wag[i] = {P_.wagC1 * drive, P_.wagC2 * drive};
      ust_.kus[i] = {0.5 * lt.a_wav[i], 0.5 * lt.a_wav[i]};
    }
  }
  ust_.gam = lt.gam;
}

void Craft::reset(const State & s, const std::array<double, 3> & delta)
{
  act_ = delta;
  resetFilters(s);
}

void Craft::setPhysics(const Physics & ph, const State & s)
{
  P_.phys = ph;
  resetFilters(s);
}

Wrench Craft::step(
  const State & s, const std::array<double, 3> & delta_cmd, double rpm, double t, double dt,
  const Sea & sea)
{
  std::array<double, 3> cmd;
  for (int i = 0; i < 3; ++i) {cmd[i] = std::clamp(delta_cmd[i], -P_.max_deflect, P_.max_deflect);}
  const Deriv D = deriv(s, act_, ust_, cmd, rpm, t, sea);
  // exact discretisation of first-order lags over dt (inputs held):
  // x' = k (target - x)  ->  x += (1 - exp(-k dt)) / k * x'
  auto adv = [dt](double & x, double dx, double k) {
      x += k > 0 ? (1 - std::exp(-k * dt)) / k * dx : dt * dx;
    };
  const double ka = 1 - std::exp(-dt / P_.act_tau);
  for (int i = 0; i < 3; ++i) {
    double step = ka * (cmd[i] - act_[i]);
    if (P_.phys.actuator_rate) {step = std::clamp(step, -P_.act_rate * dt, P_.act_rate * dt);}
    act_[i] += step;
    for (int j = 0; j < 2; ++j) {
      adv(ust_.wag[i][j], D.dx.wag[i][j], D.pole.wag[i][j]);
      adv(ust_.kus[i][j], D.dx.kus[i][j], D.pole.kus[i][j]);
    }
    adv(ust_.vent[i], D.dx.vent[i], D.pole.vent[i]);
    ust_.vent[i] = std::clamp(ust_.vent[i], 0.0, 1.0);
  }
  for (int i = 0; i < 2; ++i) {adv(ust_.gam[i], D.dx.gam[i], D.pole.gam[i]);}
  return applyAddedMass(s, D.W);
}

}  // namespace hf
