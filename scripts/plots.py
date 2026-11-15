import sys, pathlib
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
F = ROOT / "figures"
C = {"LQI-SW": "#444444", "MPC-I": "#9ecae1", "MPC-ID": "#4292c6", "MPC-IP": "#e6550d",
     "MPC-IDP": "#31a354", "LPV-MPC-IDP": "#756bb1", "LQI": "#bbbbbb", "LQI-S": "#888888"}

# step
L = np.load(F / "step_logs_t0_0.5.npy", allow_pickle=True).item()
fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
for n in ("LQI-SW", "MPC-ID", "MPC-IP", "MPC-IDP"):
    l = L[n]
    ax[0].plot(l["t"], -l["z"], color=C[n], label=n)
    ax[1].plot(l["t"], np.rad2deg(l["theta"]), color=C[n])
    ax[2].step(l["t"], np.rad2deg(l["cmd"][:, 0]), color=C[n], where="post")
ax[0].plot(L["LQI-SW"]["t"], -L["LQI-SW"]["zref"], "k--", lw=1, label="reference")
ax[0].set_ylabel("CG height above SWL [m]"); ax[1].set_ylabel("pitch [deg]")
ax[2].set_ylabel("fore-stbd cmd [deg]"); ax[2].set_xlabel("time [s]")
ax[0].legend(ncol=3, fontsize=8); [a.grid(alpha=.3) for a in ax]
ax[0].set_title("+0.2 m ride-height step at 7 m/s (Python port, nonlinear plant w/ Wagner + actuator lag)")
fig.tight_layout(); fig.savefig(F / "step_response.png", dpi=130)

# waves
p = F / "wave_sweep.npy"
if p.exists():
    R = np.load(p, allow_pickle=True).item()
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for (n, lab), r in R.items():
        ls = "-" if lab == "head" else "--"
        ax[0].plot(r[:, 1], r[:, 2], ls, marker="o", ms=3, color=C[n], label=f"{n} {lab}")
        ax[1].plot(r[:, 1], r[:, 3], ls, marker="o", ms=3, color=C[n])
    ax[0].set_xlabel("encounter frequency [rad/s]"); ax[0].set_ylabel("heave amplitude / wave amplitude")
    ax[1].set_xlabel("encounter frequency [rad/s]"); ax[1].set_ylabel("pitch amplitude [deg]")
    ax[0].legend(fontsize=7); [a.grid(alpha=.3) for a in ax]
    fig.suptitle("Regular waves, zeta_a = 0.1 m, 7 m/s (cf. thesis Fig. 5.8)")
    fig.tight_layout(); fig.savefig(F / "wave_response.png", dpi=130)

# speed ramp
p = F / "speed_ramp.npy"
if p.exists():
    S = np.load(p, allow_pickle=True).item()
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for n, l in S.items():
        ax[0].plot(l["t"], -l["z"], color=C[n], label=n)
        ax[1].plot(l["t"], l["u"], color=C[n])
        ax[2].step(l["t"], np.rad2deg(l["cmd"][:, 0]), color=C[n], where="post")
    l = next(iter(S.values()))
    ax[0].plot(l["t"], -l["zref"], "k--", lw=1, label="reference")
    ax[0].set_ylabel("CG height [m]"); ax[1].set_ylabel("speed u [m/s]"); ax[2].set_ylabel("fore-stbd cmd [deg]")
    ax[2].set_xlabel("time [s]"); ax[0].legend(fontsize=8); [a.grid(alpha=.3) for a in ax]
    ax[0].set_title("reftrack.m scenario: 7 -> 10 m/s ramp + 0.1 m height change")
    fig.tight_layout(); fig.savefig(F / "speed_ramp.png", dpi=130)
print("saved")
