"""Phase 2 scenario tests for the Supervisor (speed + height MPC, operator
overrides) on the nonlinear Python plant."""
import sys, pathlib, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from hydrofoil.design import build_models, NOMINAL_IDX
from hydrofoil.supervisor import Supervisor
from hydrofoil.sim import simulate_supervised
from hydrofoil.waves import irregular_ittc

models = build_models(); nom = models[NOMINAL_IDX]
R = {}

def run(name, T, events, waves=None):
    sup = Supervisor(models)
    tic = time.time()
    lg = simulate_supervised(sup, nom, T=T, events=events, waves=waves)
    R[name] = lg
    return sup, lg, time.time() - tic

def band(lg, t0, t1, key, sc=1.0):
    m = (lg["t"] >= t0) & (lg["t"] <= t1)
    return lg[key][m] * sc

# 1. hold: default design speed, nothing commanded
sup, lg, dt = run("hold", 6.0, [])
print(f"1 hold 7 m/s          : speed err max {np.abs(lg['u']-7).max():.4f} m/s, height err max {100*np.abs(-lg['z']-0.3).max():.3f} cm  failed={lg['failed']} ({dt:.1f}s)")

# 2. speed 7 -> 11 at 0.8 m/s^2, back to 6 at 0.5
sup, lg, dt = run("speed", 22.0, [(1.0, lambda s: (s.set_accel(0.8, 0.5), s.set_speed(11.0))),
                                  (10.0, lambda s: s.set_speed(6.0))])
acc = band(lg, 2.0, 5.5, "accel"); dec = band(lg, 11.5, 19.0, "accel")
print(f"2 speed 7->11->6      : accel phase mean {acc.mean():.3f} (limit 0.8), decel phase mean {dec.mean():.3f} (limit -0.5); "
      f"speed at 9.5s {lg['u'][int(9.5/0.005)]:.3f}, final {lg['u'][-1]:.3f}; height err max {100*np.abs(lg['z']-lg['zref']).max():.2f} cm; "
      f"pitch max {np.rad2deg(np.abs(lg['theta']).max()):.2f} deg  failed={lg['failed']}")
print(f"                        speed tracking err during ramps: max {np.abs(lg['u']-lg['uref']).max():.3f} m/s")

# 3. ride height 0.30 -> 0.45 -> 0.20
sup, lg, dt = run("height", 12.0, [(1.0, lambda s: s.set_height(0.45)), (6.0, lambda s: s.set_height(0.20))])
print(f"3 height .30->.45->.20: at 5.5s {-lg['z'][int(5.5/0.005)]:.4f}, final {-lg['z'][-1]:.4f}; speed dev max {np.abs(lg['u']-7).max():.3f} m/s; min foil submergence {lg['hmin'].min():.3f} m  failed={lg['failed']}")

# 4. ASSIST: operator fixes the aft foil 1 deg above its trim, MPC keeps flying
aft_trim = np.rad2deg(nom["delta"][2])
sup, lg, dt = run("assist", 12.0, [(1.0, lambda s: (s.set_mode("ASSIST"), s.set_override([None, None, aft_trim + 1.0]))),
                                   (7.0, lambda s: s.set_mode("AUTO"))])
m = (lg["t"] > 4) & (lg["t"] < 7)
print(f"4 ASSIST aft +1 deg   : aft actual {np.rad2deg(lg['act'][m, 2]).mean():.3f} deg (cmd {aft_trim+1:.3f}); height {-lg['z'][m].mean():.3f} m, pitch {np.rad2deg(lg['theta'][m]).mean():+.2f} deg, "
      f"speed {lg['u'][m].mean():.3f}; after release: height {-lg['z'][-1]:.4f}, pitch {np.rad2deg(lg['theta'][-1]):+.3f}; mode end {sup.mode}; failsafe '{sup.failsafe}'  failed={lg['failed']}")

# 5. MANUAL with a bad command: operator pushes all foils to 8 deg -> watchdog must take over
sup, lg, dt = run("manual_bad", 10.0, [(1.0, lambda s: (s.set_mode("MANUAL"), s.set_override([8, 8, 8])))])
print(f"5 MANUAL foils=8 deg  : mode end {sup.mode}; failsafe '{sup.failsafe}'; max height {(-lg['z']).max():.3f} m, min submergence {lg['hmin'].min():.3f} m; final height {-lg['z'][-1]:.3f}, speed {lg['u'][-1]:.3f}  failed={lg['failed']}")

# 6. MANUAL then bumpless return to AUTO (operator holds trim angles 3 s, then hands back)
tr = np.rad2deg(nom["delta"])
sup, lg, dt = run("manual_ok", 9.0, [(1.0, lambda s: (s.set_mode("MANUAL"), s.set_override(list(tr)))),
                                     (2.5, lambda s: s.set_mode("AUTO"))])
m = (lg["t"] > 2.5) & (lg["t"] < 3.5)
jump = np.abs(np.diff(np.rad2deg(lg["cmd"][m, 0]))).max()
print(f"6 MANUAL->AUTO        : drift while manual {100*(-lg['z'][int(2.5/0.005)]-0.3):+.2f} cm; largest foil cmd jump at hand-back {jump:.2f} deg; final height {-lg['z'][-1]:.4f}; failsafe '{sup.failsafe}'  failed={lg['failed']}")

# 7. everything at once in a seaway: Hs 0.3 m head sea, accelerate to 10, raise to 0.4
wv = irregular_ittc(0.3, 3.0, psi_mean=np.pi, n_freq=30, seed=3)
sup, lg, dt = run("seaway", 25.0, [(2.0, lambda s: (s.set_accel(0.6), s.set_speed(10.0))),
                                   (8.0, lambda s: s.set_height(0.40))], waves=wv)
m = lg["t"] > 14
print(f"7 Hs=0.3 m head sea   : speed mean {lg['u'][m].mean():.3f} (set 10), height mean {-lg['z'][m].mean():.3f} std {100*lg['z'][m].std():.2f} cm, pitch std {np.rad2deg(lg['theta'][m].std()):.2f} deg, "
      f"min submergence {lg['hmin'].min():.3f} m; mode {sup.mode}; failsafe '{sup.failsafe}'  failed={lg['failed']} ({dt:.1f}s)")
np.save(ROOT / "figures" / "phase2_logs.npy", R, allow_pickle=True)
