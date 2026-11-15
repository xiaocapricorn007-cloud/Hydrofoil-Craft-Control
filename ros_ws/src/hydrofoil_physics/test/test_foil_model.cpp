// Checks libfoil against golden values exported from the validated Python model.
#include <cmath>
#include <cstdio>
#include <algorithm>

#include "golden.hpp"
#include "hydrofoil/foil_model.hpp"

int main()
{
  hf::Params P;
  P.wagner = false;
  hf::Craft craft(P);
  double worst = 0.0;
  int bad = 0;
  for (int i = 0; i < hf::golden::N; ++i) {
    const double * c = hf::golden::CASES[i];
    hf::State s;
    s.xN = c[0]; s.z = c[1]; s.phi = c[2]; s.theta = c[3];
    s.u = c[4]; s.w = c[5]; s.p = c[6]; s.q = c[7];
    std::array<double, 3> d{c[8], c[9], c[10]};
    hf::WaveField wf;
    if (c[13] > 0) {wf.add(c[13], c[14], c[15], c[16]);}
    hf::Sea sea(wf);
    hf::Env & E = craft.env();
    E.rho = c[24]; E.cur_n = c[25]; E.cur_e = c[26]; E.wind_n = c[27]; E.wind_e = c[28];
    E.aero = c[29] > 0.5;
    std::array<double, 3> al{};
    hf::Wrench W = craft.forces(s, d, c[11], c[12], sea, false, &al);
    const double got[7] = {W.X, W.Z, W.K, W.M, al[0], al[1], al[2]};
    for (int k = 0; k < 7; ++k) {
      const double want = c[17 + k];
      const double err = std::abs(got[k] - want) / std::max(1.0, std::abs(want));
      worst = std::max(worst, err);
      if (err > 1e-9) {
        if (bad < 10) {std::printf("case %d col %d: got %.12g want %.12g\n", i, k, got[k], want);}
        ++bad;
      }
    }
  }
  // blended sea: field A from t = 0 (ramp 5 s), replaced by B at t = 10 (ramp 4 s)
  hf::WaveField A, B;
  A.add(0.1, 3.0, M_PI, 0.4);
  B.add(0.2, 1.8, 0.0, 1.1);
  hf::Sea sea;
  double worstB = 0.0;
  bool setB = false;
  sea.set(A, 0.0, 5.0);
  for (int i = 0; i < hf::golden::NB; ++i) {
    const double * b = hf::golden::BLEND[i];
    if (!setB && b[0] >= 10.0) {sea.set(B, 10.0, 4.0); setB = true;}
    double z; hf::Vec3 v;
    sea.sample(3.0, 1.0, 0.4, b[0], 9.81, z, v);
    const double got[4] = {z, v.x, v.y, v.z};
    for (int k = 0; k < 4; ++k) {
      const double err = std::abs(got[k] - b[1 + k]);
      worstB = std::max(worstB, err);
      if (err > 1e-12) {std::printf("blend t=%.2f col %d: got %.12g want %.12g\n", b[0], k, got[k], b[1 + k]); ++bad;}
    }
  }
  // fast grid evaluation must equal point sampling
  {
    std::vector<float> grid;
    const int nx = 7, ny = 5;
    sea.elevationGrid(2.0, -1.5, 0.6, -0.6, nx, ny, 12.0, 9.81, grid);
    double worstG = 0.0;
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        double z; hf::Vec3 v;
        sea.sample(2.0 + 0.6 * i, -1.5 - 0.6 * j, 0.0, 12.0, 9.81, z, v);
        worstG = std::max(worstG, std::abs(z - grid[j * nx + i]));
      }
    }
    if (worstG > 1e-6) {std::printf("elevationGrid mismatch %.3e\n", worstG); ++bad;}
  }
  // actuator lag: exact first-order response, 63.2 % after one time constant
  hf::Params P2;
  hf::Craft c2(P2);
  hf::State s0; s0.z = -0.3; s0.u = 7.0;
  c2.reset(s0, {0.0, 0.0, 0.0});
  hf::Sea calm;
  for (int k = 0; k < 100; ++k) {c2.step(s0, {0.1, 0.1, 0.1}, 1170.0, k * 1e-3, 1e-3, calm);}
  const double frac = c2.actuators()[0] / 0.1;
  if (std::abs(frac - (1 - std::exp(-1.0))) > 1e-9) {std::printf("actuator lag wrong: %.6f\n", frac); ++bad;}
  // ---------------------------------------------------------------- phase 4
  double worst4 = 0.0;
  {
    hf::Params P0;
    for (int k = 0; k < 9; ++k) {
      const double err = std::abs(P0.MA[k / 3][k % 3] - hf::golden::MA[k]) / std::max(1.0, std::abs(hf::golden::MA[k]));
      if (err > 1e-12) {std::printf("MA[%d] got %.12g want %.12g\n", k, P0.MA[k / 3][k % 3], hf::golden::MA[k]); ++bad;}
    }
    for (int k = 0; k < 3; ++k) {
      if (std::abs(P0.A33[k] - hf::golden::A33[k]) > 1e-12) {std::printf("A33[%d] wrong\n", k); ++bad;}
    }
  }
  for (int i = 0; i < hf::golden::N4; ++i) {
    const double * c = hf::golden::CASES4[i];
    hf::Params Pp;
    Pp.phys = hf::Physics::fromMask(static_cast<int>(c[43]));
    hf::Craft cr(Pp);
    hf::State s;
    s.xN = c[0]; s.z = c[1]; s.phi = c[2]; s.theta = c[3];
    s.u = c[4]; s.w = c[5]; s.p = c[6]; s.q = c[7];
    std::array<double, 3> act{c[8], c[9], c[10]};
    hf::Unsteady U;
    for (int f = 0; f < 3; ++f) {
      U.wag[f] = {c[11 + 2 * f], c[12 + 2 * f]};
      U.kus[f] = {c[17 + 2 * f], c[18 + 2 * f]};
      U.vent[f] = c[23 + f];
    }
    U.gam = {c[26], c[27]};
    std::array<double, 3> cmd{c[28], c[29], c[30]};
    hf::WaveField wf;
    if (c[33] > 0) {wf.add(c[33], c[34], c[35], c[36]);}
    hf::Sea sea(wf);
    hf::Env & E = cr.env();
    E.rho = c[37]; E.cur_n = c[38]; E.cur_e = c[39]; E.wind_n = c[40]; E.wind_e = c[41];
    E.aero = c[42] > 0.5;
    const hf::Deriv D = cr.deriv(s, act, U, cmd, c[31], c[32], sea);
    const auto a = cr.accelWPQ(s, D.W);
    double got[30] = {D.W.X, D.W.Z, D.W.K, D.W.M, a[0], a[1], a[2], D.dact[0], D.dact[1], D.dact[2]};
    for (int f = 0; f < 3; ++f) {
      got[10 + 2 * f] = D.dx.wag[f][0]; got[11 + 2 * f] = D.dx.wag[f][1];
      got[16 + 2 * f] = D.dx.kus[f][0]; got[17 + 2 * f] = D.dx.kus[f][1];
      got[22 + f] = D.dx.vent[f];
    }
    got[25] = D.dx.gam[0]; got[26] = D.dx.gam[1];
    for (int k = 0; k < 27; ++k) {
      const double want = c[44 + k];
      const double err = std::abs(got[k] - want) / std::max(1.0, std::abs(want));
      worst4 = std::max(worst4, err);
      if (err > 1e-9) {
        if (bad < 20) {
          std::printf("p4 case %d (mask %d) col %d: got %.12g want %.12g\n", i, static_cast<int>(c[43]), k, got[k], want);
        }
        ++bad;
      }
    }
    // the transformed wrench must give a plain rigid body the same accelerations
    const hf::Wrench Wp = cr.applyAddedMass(s, D.W);
    const double G = Pp.mass * Pp.g * std::cos(s.theta) * std::cos(s.phi);
    const double ap[3] = {(Wp.Z + G) / Pp.mass + s.q * s.u, Wp.K / Pp.Ixx, Wp.M / Pp.Iyy};
    for (int k = 0; k < 3; ++k) {
      if (std::abs(ap[k] - a[k]) > 1e-9 * std::max(1.0, std::abs(a[k]))) {
        std::printf("p4 case %d: added-mass transform axis %d: %.12g vs %.12g\n", i, k, ap[k], a[k]);
        ++bad;
      }
    }
  }
  // trims
  double worstT = 0.0;
  for (int i = 0; i < hf::golden::NT; ++i) {
    const double * r = hf::golden::TRIMS[i];
    hf::Params Pt;
    Pt.wagner = false;
    Pt.phys = hf::Physics::fromMask(static_cast<int>(r[0]));
    hf::Craft ct(Pt);
    std::array<double, 3> d{0.05, 0.05, 0.05};
    double rpmT = 1500.0;
    if (!ct.trim(r[1], -0.3, d, rpmT)) {std::printf("trim failed mask %d U %.1f\n", static_cast<int>(r[0]), r[1]); ++bad; continue;}
    const double e = std::max({std::abs(d[0] - r[2]), std::abs(d[2] - r[3]), std::abs(rpmT - r[4]) / 1000.0});
    worstT = std::max(worstT, e);
    if (e > 1e-7) {std::printf("trim mask %d U %.1f: %.9f %.9f %.4f vs %.9f %.9f %.4f\n", static_cast<int>(r[0]), r[1], d[0], d[2], rpmT, r[2], r[3], r[4]); ++bad;}
  }
  // physics names
  {
    hf::Physics ph;
    if (!hf::Physics::parse("downwash, ventilation", ph) || ph.mask() != 48) {std::printf("parse names failed\n"); ++bad;}
    if (!hf::Physics::parse("v2", ph) || ph.mask() != 127 || ph.str() != "v2") {std::printf("parse v2 failed\n"); ++bad;}
    if (hf::Physics::parse("warp_drive", ph)) {std::printf("parse accepted junk\n"); ++bad;}
  }
  // rate limit: a 0.2 rad command step moves at most act_rate * t
  {
    hf::Params Pr;
    Pr.phys.actuator_rate = true;
    hf::Craft cr(Pr);
    hf::State s0; s0.z = -0.3; s0.u = 7.0;
    cr.reset(s0, {0.0, 0.0, 0.0});
    hf::Sea calm;
    for (int k = 0; k < 50; ++k) {cr.step(s0, {0.17, 0.17, 0.17}, 1170.0, k * 1e-3, 1e-3, calm);}
    if (std::abs(cr.actuators()[0] - 0.05 * Pr.act_rate) > 1e-9) {
      std::printf("rate limit wrong: %.6f vs %.6f\n", cr.actuators()[0], 0.05 * Pr.act_rate); ++bad;
    }
  }
  std::printf(
    "libfoil golden test: %d force cases (worst rel. error %.3e), %d sea-blend cases (worst abs. error %.3e),\n"
    "  %d phase-4 derivative cases (worst rel. error %.3e), %d trims (worst %.2e), failures %d\n",
    hf::golden::N, worst, hf::golden::NB, worstB, hf::golden::N4, worst4, hf::golden::NT, worstT, bad);
  return bad == 0 ? 0 : 1;
}
