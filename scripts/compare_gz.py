"""Compare a Gazebo parity log against the Python simulator on the same
scenario (+0.2 m step at t = 0.5 s, 7 m/s).
   python scripts/compare_gz.py MPC-IDP /tmp/gz_step_MPC-IDP.npz"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hydrofoil.design import build_models, build_controllers, NOMINAL_IDX
from hydrofoil.sim import simulate, metrics, make_reference

name, path = sys.argv[1], sys.argv[2]
g = dict(np.load(path)); g["failed"] = False
models = build_models(); nom = models[NOMINAL_IDX]
c = build_controllers(models, which=(name,))[name]
T = float(g["t"][-1])
p = simulate(c, nom, T=round(T, 1), ref=make_reference("step", dz=-0.2, t0=0.5))
mg, mp = metrics(g, nom["delta"]), metrics(p, nom["delta"])
print(f"{name}: {len(g['t'])} Gazebo samples over {T:.2f} s")
print(f"{'metric':8s} {'gazebo':>9s} {'python':>9s} {'diff %':>8s}")
for k in ("ISE", "IAE", "ITAE", "effort"):
    print(f"{k:8s} {mg[k]:9.4f} {mp[k]:9.4f} {100*(mg[k]-mp[k])/mp[k]:8.2f}")
for k, sc, unit in (("z", 100, "cm"), ("theta", 180 / np.pi, "deg"), ("phi", 180 / np.pi, "deg"), ("u", 1, "m/s")):
    gi = np.interp(p["t"], g["t"], g[k])
    d = (gi - p[k]) * sc
    print(f"trajectory diff {k:6s}: max |d| = {np.abs(d).max():.4f} {unit}, rms = {np.sqrt((d**2).mean()):.4f} {unit}")
if "v" in g:
    print(f"lateral lock: max |v| = {np.abs(g['v']).max():.2e} m/s, max |r| = {np.abs(g['r']).max():.2e} rad/s, "
          f"max |psi| = {np.rad2deg(np.abs(g['psi']).max()):.2e} deg")
