"""Does controller latency explain the Gazebo-vs-Python difference in waves?
Re-run the Python simulator with the command applied 0, 5, 10, ... ms late and
compare each against the Gazebo log of the same scenario.
   python3 scripts/latency_probe.py scenarios/sea_head.json /tmp/gz_phase2_sea_head.npz"""
import sys, json, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hydrofoil.design import build_models, NOMINAL_IDX
from hydrofoil.supervisor import Supervisor
from hydrofoil.environment import Environment
from hydrofoil.sim import simulate_supervised

scen = json.load(open(sys.argv[1]))
g = dict(np.load(sys.argv[2]))
T = round(float(g["t"][-1]), 1)
models = build_models(); nom = models[NOMINAL_IDX]


def act(name, value):
    def f(s, env, t):
        if name == "env": env.apply(value, t)
        elif name == "speed": s.set_speed(value)
        elif name == "accel": s.set_accel(*(value if isinstance(value, list) else [value]))
        elif name == "height": s.set_height(value)
        elif name == "mode": s.set_mode(value)
        elif name == "foil": s.set_override(value)
    return f


print(f"Gazebo log: {len(g['t'])} samples, z std {100*g['z'].std():.2f} cm, pitch std {np.rad2deg(g['theta'].std()):.3f} deg")
print(f"{'delay':>8s} {'z rms diff':>11s} {'theta rms diff':>15s} {'python z std':>13s} {'python theta std':>17s}")
for steps in (0, 1, 2, 3, 4, 6, 8):
    sup, env = Supervisor(models), Environment(seed=0, aero=True)
    p = simulate_supervised(sup, nom, T=T, events=[(e[0], act(e[1], e[2])) for e in scen], env=env,
                            cmd_delay_steps=steps)
    dz = (np.interp(p["t"], g["t"], g["z"]) - p["z"]) * 100
    dth = np.rad2deg(np.interp(p["t"], g["t"], g["theta"]) - p["theta"])
    print(f"{steps*5:5d} ms {np.sqrt((dz**2).mean()):9.3f} cm {np.sqrt((dth**2).mean()):12.4f} deg "
          f"{100*p['z'].std():10.2f} cm {np.rad2deg(p['theta'].std()):14.3f} deg")
