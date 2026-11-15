"""Phase 3 tests on the nonlinear Python plant: the operator changes the
environment while the Supervisor (speed + height MPC) is flying."""
import sys, pathlib, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from hydrofoil.design import build_models, NOMINAL_IDX
from hydrofoil.supervisor import Supervisor
from hydrofoil.environment import Environment
from hydrofoil.sim import simulate_supervised

models = build_models(); nom = models[NOMINAL_IDX]
R = {}


def E(upd):
    return lambda sup, env, t: env.apply(upd, t)


def run(name, T, events, aero=True, seed=0):
    sup, env = Supervisor(models), Environment(seed=seed, aero=aero)
    tic = time.time()
    lg = simulate_supervised(sup, nom, T=T, events=events, env=env)
    R[name] = lg
    return sup, env, lg, time.time() - tic


def win(lg, t0, t1):
    return (lg["t"] >= t0) & (lg["t"] <= t1)


def line(tag, lg, sup, t0, t1, extra=""):
    m = win(lg, t0, t1)
    print(f"{tag:26s}: speed {lg['u'][m].mean():6.3f} (std {lg['u'][m].std():.3f}) | height {(-lg['z'][m]).mean():.3f} "
          f"std {100*lg['z'][m].std():5.2f} cm | pitch std {np.rad2deg(lg['theta'][m].std()):.2f} roll std {np.rad2deg(lg['phi'][m].std()):.2f} deg | "
          f"min sub {lg['hmin'][m].min():.3f} m | rpm {lg['rpm'][m].mean():5.0f} | foils {np.rad2deg(lg['act'][m].mean(0)).round(2)} "
          f"| {sup.mode} {('!'+sup.failsafe) if sup.failsafe else ''} failed={lg['failed']} {extra}")


# 0. air drag on, calm: how far does the trim move from the reference?
sup, env, lg, dt = run("aero", 10.0, [])
line("0 calm, air drag on", lg, sup, 6, 10)

# 1. water density: sea water -> fresh water (rho 1024.8 -> 997) at t = 3
sup, env, lg, dt = run("fresh", 14.0, [(3.0, E({"water": {"temp": 25, "salinity": 0}}))])
m = win(lg, 3, 6)
line("1 fresh water at t=3", lg, sup, 9, 14, f"| transient: max height err {100*np.abs(lg['z'][m]-lg['zref'][m]).max():.2f} cm, rho now {env.water['rho']:.1f}")

# 2. head wind 12 m/s with gusts at t = 3
sup, env, lg, dt = run("headwind", 20.0, [(3.0, E({"wind": {"speed": 12, "dir": 0, "gust": 0.25}}))])
line("2 headwind 12 m/s gusty", lg, sup, 8, 20)

# 3. beam wind 12 m/s from starboard
sup, env, lg, dt = run("beamwind", 20.0, [(3.0, E({"wind": {"speed": 12, "dir": 90, "gust": 0.25}}))])
line("3 beam wind 12 m/s gusty", lg, sup, 8, 20, f"| mean roll {np.rad2deg(lg['phi'][win(lg, 8, 20)].mean()):+.3f} deg")

# 4. current 1.5 m/s against the craft
sup, env, lg, dt = run("current", 16.0, [(3.0, E({"current": {"speed": 1.5, "dir": 180}}))])
line("4 current 1.5 m/s against", lg, sup, 10, 16, "| speed is over ground; through water = +1.5")

# 5. sea state ladder, head seas: calm -> Hs 0.2 -> 0.4 -> 0.6
ev = [(2.0, E({"sea": {"hs": 0.2, "tp": 2.5, "dir": 0}})), (22.0, E({"sea": {"hs": 0.4, "tp": 3.0, "dir": 0}})),
      (42.0, E({"sea": {"hs": 0.6, "tp": 3.5, "dir": 0}}))]
sup, env, lg, dt = run("ladder_head", 62.0, ev)
for tag, a, b in (("5 head sea Hs 0.2 Tp 2.5", 8, 22), ("5 head sea Hs 0.4 Tp 3.0", 28, 42), ("5 head sea Hs 0.6 Tp 3.5", 48, 62)):
    line(tag, lg, sup, a, b)
print(f"   (62 s of simulation took {dt:.1f} s)")

# 6. same ladder, following and beam seas at Hs 0.4
for name, d in (("following", 180), ("beam", 90), ("bow quarter", 45)):
    sup, env, lg, dt = run("sea_" + name, 30.0, [(2.0, E({"sea": {"hs": 0.4, "tp": 3.0, "dir": d, "spread": name != "following"}}))])
    line(f"6 Hs 0.4 {name}", lg, sup, 10, 30)

# 7. everything: Hs 0.4 head sea + 10 m/s gusty wind from 30 deg + 0.5 m/s current + speed change + ASSIST
ev = [(1.0, E({"sea": {"hs": 0.4, "tp": 3.0, "dir": 20, "spread": True}, "wind": {"speed": 10, "dir": 30, "gust": 0.3},
               "current": {"speed": 0.5, "dir": 200}})),
      (8.0, lambda s: (s.set_accel(0.6), s.set_speed(10.0), s.set_height(0.38)))]
sup, env, lg, dt = run("combined", 40.0, ev)
line("7 combined, 10 m/s", lg, sup, 20, 40)
np.save(ROOT / "figures" / "phase3_logs.npy", R, allow_pickle=True)
