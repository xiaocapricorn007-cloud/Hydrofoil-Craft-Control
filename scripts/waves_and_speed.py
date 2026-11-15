"""(1) Regular-wave response sweep, zeta_a = 0.1 m, head & following seas, 7 m/s
    (thesis Sec. 5.3.2 / Fig. 5.8).  Amplitude measured as in measureAmplitude.m
    (mean peak-to-trough / 2 after discarding the first 20 %).
(2) reftrack.m scenario: speed ramp 7 -> 10 m/s (t = 1..3 s) + smooth +0.1 m
    ride-height change (t = 2..3 s), 6 s."""
import sys, pathlib
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from scipy.signal import find_peaks
from hydrofoil.design import build_models, build_controllers, NOMINAL_IDX
from hydrofoil.sim import simulate, metrics, make_reference, smoothstep
from hydrofoil.waves import regular

models = build_models(); nom = models[NOMINAL_IDX]

def amplitude(t, y, frac=0.2):
    y = y[int(len(y) * frac):]; y = y - y.mean()
    a, _ = find_peaks(y); b, _ = find_peaks(-y)
    return np.nan if len(a) == 0 or len(b) == 0 else (y[a].mean() + (-y[b]).mean()) / 2

omegas = np.array([1.8, 2.5, 3.2, 4.0, 4.8, 5.6, 6.5])
res = {}
for name in ("LQI-SW", "MPC-IP", "MPC-IDP"):
    for psi, lab in ((np.pi, "head"), (0.0, "following")):
        rows = []
        for w in omegas:
            c = build_controllers(models, which=(name,))[name]
            wv = regular(0.1, w, psi)
            we = abs(w - w ** 2 * 7.0 / 9.81 * np.cos(psi))      # encounter freq (2.4.13)
            T = max(12.0, 8 * 2 * np.pi / max(we, 0.3))
            lg = simulate(c, nom, T=min(T, 40.0), waves=wv)
            if lg["failed"]:
                rows.append((w, we, np.nan, np.nan, True)); continue
            rows.append((w, we, amplitude(lg["t"], lg["z"]) / 0.1,
                         np.rad2deg(amplitude(lg["t"], lg["theta"])), False))
            print(f"{name:8s} {lab:9s} w={w:.1f} we={we:5.2f}  z/za={rows[-1][2]:.3f}  theta={rows[-1][3]:.3f} deg", flush=True)
        res[(name, lab)] = np.array(rows, dtype=float)
np.save(ROOT / "figures" / "wave_sweep.npy", res, allow_pickle=True)

# ---- speed change (reftrack.m) ------------------------------------------
zref = lambda t: np.r_[-0.3 + smoothstep(t, 2.0, 1.0, -0.1), np.zeros(5)]
uref = lambda t: 7.0 + 3.0 * np.clip((t - 1.0) / 2.0, 0, 1)
spd = {}
for name in ("LQI-SW", "MPC-IP", "MPC-IDP", "LPV-MPC-IDP"):
    c = build_controllers(models, which=(name,))[name]
    lg = simulate(c, nom, T=6.0, ref=zref, speed_ref=uref)
    m = metrics(lg, nom["delta"])
    spd[name] = lg
    print(f"speed-ramp {name:12s} ISE {m['ISE']:.3f} IAE {m['IAE']:.3f} effort(vs 7 m/s trim) {m['effort']:.3f} "
          f"final z err {100*(lg['z'][-1]-lg['zref'][-1]):+.2f} cm  failed={m['failed']}")
np.save(ROOT / "figures" / "speed_ramp.npy", spd, allow_pickle=True)
