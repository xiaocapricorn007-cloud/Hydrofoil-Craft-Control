"""Run a scenario file through the Python simulator and compare with the
Gazebo log of the same scenario.
   python scripts/compare_phase2.py scenarios/tour.json logs/gz_phase2_tour.npz [seed] [aero 0/1] [physics]
physics: ref (default) | v2 | mask | switch names -- must match the Gazebo run.
Scenario actions: speed, accel, height, pitch, heel, mode, foil (operator) and
env (a dict for hydrofoil.environment.Environment.apply)."""
import sys, json, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hydrofoil.design import build_models, NOMINAL_IDX
from hydrofoil.supervisor import Supervisor
from hydrofoil.environment import Environment
from hydrofoil.sim import simulate_supervised
from hydrofoil.params import Params, Physics
from hydrofoil.plant import Hydrofoil
from hydrofoil.linearize import trim


def act(name, value):
    def f(s, env, t):
        if name == "speed": s.set_speed(value)
        elif name == "accel": s.set_accel(*(value if isinstance(value, list) else [value]))
        elif name == "height": s.set_height(value)
        elif name == "pitch": s.set_trim(pitch_deg=value)
        elif name == "heel": s.set_trim(heel_deg=value)
        elif name == "mode": s.set_mode(value)
        elif name == "foil": s.set_override(value)
        elif name == "env": env.apply(value, t)
    return f


scen = json.load(open(sys.argv[1]))
g = dict(np.load(sys.argv[2]))
seed = int(sys.argv[3]) if len(sys.argv) > 3 else 0
aero = bool(int(sys.argv[4])) if len(sys.argv) > 4 else True
T = round(float(g["t"][-1]), 1)
phys = Physics.parse(sys.argv[5]) if len(sys.argv) > 5 else Physics()
models = build_models(); nom = models[NOMINAL_IDX]
plant = Hydrofoil(Params(phys=phys), wagner=True)
# the Gazebo rail holds the craft at the trim of the active physics
core0, d0, rpm0 = trim(Hydrofoil(Params(phys=phys), wagner=False), 7.0, -0.3)
start = dict(nom, core=core0, delta=d0, rpm=rpm0)
sup, env = Supervisor(models), Environment(seed=seed, aero=aero)
p = simulate_supervised(sup, start, T=T, events=[(e[0], act(e[1], e[2])) for e in scen], env=env, model=plant)
print(f"physics: {phys}")
print(f"{len(g['t'])} Gazebo samples over {T} s;  python failed={p['failed']}, final mode {sup.mode}, failsafe '{sup.failsafe}'")
for k, sc, unit in (("z", 100, "cm"), ("theta", 180 / np.pi, "deg"), ("phi", 180 / np.pi, "deg"), ("u", 1, "m/s"), ("rpm", 1, "rpm")):
    d = (np.interp(p["t"], g["t"], g[k]) - p[k]) * sc
    print(f"  gazebo - python {k:6s}: max |d| = {np.abs(d).max():8.4f} {unit}   rms = {np.sqrt((d**2).mean()):8.4f} {unit}"
          f"   (signal std: gazebo {g[k].std()*sc:.4f}, python {p[k].std()*sc:.4f})")
print("  gazebo run:  speed min/max %.3f / %.3f, final %.3f | height min/max %.3f / %.3f, final %.3f | pitch max %.2f deg | roll max %.2f deg | min submergence %.3f m | final mode %d"
      % (g["u"].min(), g["u"].max(), g["u"][-1], (-g["z"]).min(), (-g["z"]).max(), -g["z"][-1],
         np.rad2deg(np.abs(g["theta"]).max()), np.rad2deg(np.abs(g["phi"]).max()), np.nanmin(g["hmin"]), g["mode"][-1]))
if "zeta" in g and np.isfinite(g["zeta"]).any() and np.nanstd(g["zeta"]) > 0:
    # wave elevation at the CG: plugin vs the Python sea state along the Gazebo track is not
    # available here, so compare statistics, and check the sonar against geometry
    print(f"  wave elevation at CG (gazebo): std {np.nanstd(g['zeta'])*100:.2f} cm, 4*std = {4*np.nanstd(g['zeta']):.3f} m")
if "sonar" in g and np.isfinite(g["sonar"]).any():
    # sonar is 0.1 m above the CG and 1.6 m ahead: range ~ (height + 0.1 + 1.6*sin(theta)) / cos + wave
    exp = (-g["z"] + 0.1 + 1.6 * np.sin(g["theta"])) / (np.cos(g["theta"]) * np.cos(g["phi"]))
    m = g["t"] > 1.0
    print(f"  sonar range: mean {np.nanmean(g['sonar'][m]):.3f} m; minus still-water geometry: mean {np.nanmean((g['sonar']-exp)[m])*100:+.2f} cm, std {np.nanstd((g['sonar']-exp)[m])*100:.2f} cm")
