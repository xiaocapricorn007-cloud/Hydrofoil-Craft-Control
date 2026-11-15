"""Thesis Sec. 5.2.1: +0.2 m ride-height step at 7 m/s, nominal model.
Compares against Tables 5.3-5.5."""
import sys, pathlib, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hydrofoil.design import build_models, build_controllers, NOMINAL_IDX
from hydrofoil.sim import simulate, metrics, make_reference

THESIS = {"LQI": (1.3105, 1.5771, 1.9944, 1.9372), "LQI-S": (1.4553, 1.6928, 2.2177, 1.4160),
          "LQI-SW": (1.2841, 1.4097, 1.6821, 1.2387),
          "MPC-I": (1.3167, 1.0583, 0.9320, 1.5396), "MPC-ID": (1.2728, 0.8512, 0.6116, 0.9284),
          "MPC-IP": (0.1304, 0.4467, 0.3185, 1.0725), "MPC-IDP": (0.1395, 0.2923, 0.1284, 0.6470)}
t0 = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
models = build_models()
nom = models[NOMINAL_IDX]
ctrls = build_controllers(models, which=("LQI", "LQI-S", "LQI-SW", "MPC-I", "MPC-ID", "MPC-IP", "MPC-IDP"))
ref = make_reference("step", dz=-0.2, t0=t0)
print(f"step at t0={t0}s, T=4 s.   port values  |  thesis values")
print(f"{'ctrl':8s} {'ISE':>7s} {'IAE':>7s} {'ITAE':>7s} {'effort':>7s} | {'ISE':>7s} {'IAE':>7s} {'ITAE':>7s} {'effort':>7s}")
logs = {}
for name, c in ctrls.items():
    tic = time.time()
    lg = simulate(c, nom, T=4.0, ref=ref)
    logs[name] = lg
    m = metrics(lg, nom["delta"])
    th = THESIS.get(name, (np.nan,) * 4)
    print(f"{name:8s} {m['ISE']:7.3f} {m['IAE']:7.3f} {m['ITAE']:7.3f} {m['effort']:7.3f} | "
          f"{th[0]:7.3f} {th[1]:7.3f} {th[2]:7.3f} {th[3]:7.3f}  {'FAILED' if m['failed'] else ''} ({time.time()-tic:.1f}s)")
np.save(pathlib.Path(__file__).resolve().parents[1] / "figures" / f"step_logs_t0_{t0}.npy", logs, allow_pickle=True)
