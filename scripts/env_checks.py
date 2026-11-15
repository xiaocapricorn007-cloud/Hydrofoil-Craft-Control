"""Sanity checks of the environment model (phase 3)."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hydrofoil.environment import Environment, seawater_density

print("seawater density: 15C/35psu = %.3f (reference value 1025.97), 25C/0psu = %.3f, 4C/35psu = %.3f"
      % (seawater_density(15, 35), seawater_density(25, 0), seawater_density(4, 35)))
e = Environment(seed=1)
for hs, tp in ((0.2, 2.0), (0.4, 3.0), (0.8, 5.0)):
    e.apply({"sea": {"hs": hs, "tp": tp, "dir": 0}}, t=0)
    st = e.status()["sea"]
    f = e.field
    t = np.arange(0, 600, 0.2)
    z = np.array([f.at(0, 0, 0, ti)[0] for ti in t])
    print(f"Hs set {hs}: from amplitudes {st['hs_realised']:.3f}, from a 600 s record 4*std = {4*z.std():.3f}; "
          f"{st['components']} components, peak omega {2*np.pi/tp:.2f} rad/s")
e.apply({"wind": {"speed": 10, "dir": 90, "gust": 0.2}})
w = np.array([e.step(k * 0.1) for k in range(3000)])
sp = np.hypot(*w.T)
print(f"wind 10 m/s from starboard, gust 0.2: mean speed {sp.mean():.2f}, std {sp.std():.2f}, "
      f"mean vector {w.mean(0).round(2)} NED (expect about [0, -10])")
# blending continuity: switch sea at t = 10 with a 5 s ramp
e2 = Environment(seed=2)
e2.apply({"sea": {"hs": 0.3, "tp": 2.5}}, t=0)
e2.apply({"sea": {"hs": 0.6, "tp": 4.0, "dir": 180}}, t=10.0)
tt = np.arange(4.99, 16.0, 0.01)
zz = np.array([e2.sea_state.at(0, 0, 0, ti)[0] for ti in tt])
print(f"sea change at t=10 s: largest elevation jump between 10 ms samples = {1000*np.abs(np.diff(zz)).max():.2f} mm")
