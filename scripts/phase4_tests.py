"""Phase 4 tests: physics upgrades on the nonlinear Python plant.

A  derived quantities (added mass, wake upwash)
B  trim of each variant (one switch on at a time, then all = v2)
C  thesis step benchmark, MPC-IDP / LQI-SW designed on the REFERENCE model,
   flown on each plant variant (robustness to the model upgrades)
D  supervised AUTO in head seas: reference plant vs v2 plant, and v2 plant with
   a controller re-designed on the v2 linearisation
E  ventilation: hysteresis check and a near-surface manoeuvre
F  actuator rate limit
"""
import sys, pathlib, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from dataclasses import replace
from hydrofoil.params import Params, Physics
from hydrofoil.plant import Hydrofoil, Unsteady, S_VENT, S_ACT
from hydrofoil.linearize import trim, linearize
from hydrofoil.design import build_models, build_controllers, NOMINAL_IDX
from hydrofoil.sim import simulate, simulate_supervised, metrics, make_reference
from hydrofoil.supervisor import Supervisor
from hydrofoil.environment import Environment
from hydrofoil.waves import calm

SW = list(vars(Physics()).keys())
VARIANTS = [("reference", Physics())] + [(k, Physics(**{k: True})) for k in SW] + [("v2 (all)", Physics.v2())]
deg = np.rad2deg


def trim_of(phys, U=7.0):
    m = Hydrofoil(Params(phys=phys), wagner=False)
    core, delta, rpm = trim(m, U, -0.3)
    return dict(core=core, delta=delta, rpm=rpm)


# ---------------------------------------------------------------- A
print("A. derived quantities")
P = Params(phys=Physics.v2())
print(f"   foil added mass A33 [kg]: {np.round(P.A33, 2)}  (craft mass {P.mass})")
print(f"   MA over [w,p,q]: diag {np.round(np.diag(P.MA), 2)}  -> +{100*P.MA[0,0]/P.mass:.1f}% heave, "
      f"+{100*P.MA[1,1]/P.Ixx:.1f}% roll, +{100*P.MA[2,2]/P.Iyy:.1f}% pitch inertia")
m = Hydrofoil(P, wagner=False)
t7 = trim_of(Physics.v2())
out = {}
m.forces(t7["core"], lambda i, a: a, t7["delta"], t7["rpm"], 0.0, calm(), out=out)
g = m._last["gam"]
print(f"   fore-foil circulation {np.round(g, 3)} m^2/s, upwash at the aft foil {out['upwash']*100:.2f} cm/s "
      f"= {deg(out['upwash']/7):.2f} deg extra incidence")
# free-surface image: phi = 0 on the surface means no tangential velocity there
from hydrofoil.wake import horseshoe
c0 = np.array(P.arm_FS); hh = 0.45; bp = np.pi / 4 * P.span
img = c0.copy(); img[2] -= 2 * hh
pts = np.array([[x, y, c0[2] - hh] for x in (-2.0, -0.5, 0.3, 2.0) for y in (0.1, 0.62, 1.2)])
vs = horseshoe(pts, c0, bp) + horseshoe(pts, img, bp)
print(f"   wake image check on the free surface: max tangential |v| {np.abs(vs[:, :2]).max():.1e} m/s "
      f"(0 = high-Froude condition), max normal |v| {np.abs(vs[:, 2]).max():.3f} m/s")
for U in (5.0, 7.0, 12.0):
    c = U / (P.mac / 2)
    print(f"   U={U:4.1f}: Wagner time constants {1e3/(0.0455*c):.1f} / {1e3/(0.3*c):.1f} ms "
          f"(reference fixes them at U=7: {1e3/(0.0455*7/(P.mac/2)):.1f} / {1e3/(0.3*7/(P.mac/2)):.1f} ms)")

# ---------------------------------------------------------------- B
print("\nB. trim at 7 m/s, CG 0.30 m above the water")
TRIMS = {}
for name, ph in VARIANTS:
    TRIMS[name] = tr = trim_of(ph)
    print(f"   {name:15s} fore {deg(tr['delta'][0]):6.3f}  aft {deg(tr['delta'][2]):6.3f} deg  rpm {tr['rpm']:7.1f}")

# linear model poles with / without added mass
for name, ph in (("reference", Physics()), ("added_mass", Physics(added_mass=True))):
    lin = Hydrofoil(Params(phys=ph), wagner=False)
    tr = TRIMS[name]
    A, B = linearize(lin, tr["core"], tr["delta"], tr["rpm"])
    ev = np.linalg.eigvals(A)
    print(f"   open-loop poles ({name}): {np.round(np.sort_complex(ev), 3)}")

# ---------------------------------------------------------------- C
print("\nC. +0.2 m ride-height step (thesis Sec 5.2.1), controllers designed on the reference model")
models = build_models(); nom = models[NOMINAL_IDX]
ctrls = build_controllers(models, which=("LQI-SW", "MPC-IDP"))
ref = make_reference("step", dz=-0.2, t0=0.5)
print(f"   {'plant':15s} {'ctrl':8s} {'ISE':>7s} {'IAE':>7s} {'ITAE':>7s} {'effort':>7s} {'max|dδ/dt|':>11s}")
STEP = {}
for name, ph in VARIANTS:
    plant = Hydrofoil(Params(phys=ph), wagner=True)
    for cn, c in ctrls.items():
        lg = simulate(c, TRIMS[name], T=4.0, ref=ref, model=plant)
        mm = metrics(lg, TRIMS[name]["delta"])
        rate = deg(np.abs(np.diff(lg["act"], axis=0)).max() / 0.005)
        STEP[(name, cn)] = lg
        print(f"   {name:15s} {cn:8s} {mm['ISE']:7.3f} {mm['IAE']:7.3f} {mm['ITAE']:7.3f} {mm['effort']:7.3f} "
              f"{rate:9.1f}°/s {'FAILED' if mm['failed'] else ''}")

# ---------------------------------------------------------------- D
print("\nD. supervised AUTO, 7 m/s, head seas (20 s window after the sea builds)")
models_v2 = build_models(params=Params(phys=Physics.v2()))


def E(upd):
    return lambda sup, env, t: env.apply(upd, t)


SEA = {}
for hs, tp in ((0.2, 2.5), (0.4, 3.0)):
    for tag, ph, mods in (("ref plant / ref MPC", Physics(), models),
                          ("v2 plant / ref MPC", Physics.v2(), models),
                          ("v2 plant / v2 MPC", Physics.v2(), models_v2)):
        sup, env = Supervisor(mods), Environment(seed=3, aero=True)
        plant = Hydrofoil(Params(phys=ph), wagner=True)
        tr = TRIMS["v2 (all)"] if ph.any() else TRIMS["reference"]
        tic = time.time()
        lg = simulate_supervised(sup, tr, T=30.0, model=plant, env=env,
                                 events=[(1.0, E({"sea": {"hs": hs, "tp": tp, "dir": 0}}))])
        SEA[(hs, tag)] = lg
        w = lg["t"] >= 10
        print(f"   Hs {hs} Tp {tp}  {tag:20s}: heave std {100*lg['z'][w].std():5.2f} cm  pitch std "
              f"{deg(lg['theta'][w].std()):.2f}  roll std {deg(lg['phi'][w].std()):.2f} deg  min sub "
              f"{lg['hmin'][w].min() if w.any() else np.nan:.3f} m  failed={lg['failed']} {sup.failsafe or sup.warning} "
              f"({time.time()-tic:.0f}s)")

# ---------------------------------------------------------------- E
print("\nE. ventilation")
# E1: hysteresis.  Hold the craft state fixed and drive the fore-foil depth down then up.
Pv = Params(phys=Physics(ventilation=True))
mv = Hydrofoil(Pv, wagner=True)
tr = TRIMS["ventilation"]
x = mv.initial_state(tr["core"], tr["delta"])
rows = []
z_path = np.r_[np.linspace(-0.30, -0.73, 200), np.linspace(-0.73, -0.30, 200)]   # CG height -> foil depth
for zc in z_path:
    x[1] = zc
    for _ in range(20):                       # 0.1 s at each depth
        dx = mv.deriv(x, tr["delta"], tr["rpm"], 0.0, calm())
        x[S_VENT] += 0.005 * dx[S_VENT]
    out = {}
    mv.forces(x[:8], None, x[S_ACT], tr["rpm"], 0.0, calm(),
              ust=Unsteady(x[11:17].reshape(3, 2), x[17:23].reshape(3, 2), x[S_VENT], x[26:28]), out=out)
    rows.append((out["FS"]["h"], x[S_VENT][0], out["FS"]["L"]))
rows = np.array(rows)
down, up = rows[:200], rows[200:]
on = down[np.argmax(down[:, 1] > 0.5), 0]
off = up[np.argmax(up[:, 1] < 0.5), 0]
print(f"   fore foil ventilates going shallower at h = {100*on:.1f} cm, re-attaches going deeper at h = {100*off:.1f} cm"
      f" (set: {100*Pv.vent_h_on*Pv.mac:.0f} / {100*Pv.vent_h_off*Pv.mac:.0f} cm)")
print(f"   lift at h=3 cm: wetted branch n/a, ventilated {down[np.argmin(abs(down[:,0]-0.03)),2]:.0f} N; "
      f"at h=8 cm descending {down[np.argmin(abs(down[:,0]-0.08)),2]:.0f} N vs ascending {up[np.argmin(abs(up[:,0]-0.08)),2]:.0f} N")
# E2: closed loop, operator asks for a high ride height in Hs 0.5 seas
for tag, ph in (("reference plant", Physics()), ("ventilation only", Physics(ventilation=True)),
                ("v2 plant", Physics.v2())):
    sup, env = Supervisor(models), Environment(seed=5, aero=True)
    plant = Hydrofoil(Params(phys=ph), wagner=True)
    lg = simulate_supervised(sup, TRIMS["v2 (all)" if ph.downwash else "reference"], T=40.0, model=plant, env=env,
                             events=[(1.0, E({"sea": {"hs": 0.5, "tp": 3.0, "dir": 0}})),
                                     (5.0, lambda s: s.set_height(0.55))])
    w = lg["t"] >= 15
    print(f"   Hs 0.5, ride height 0.55 m, {tag:17s}: min sub {lg['hmin'][w].min():.3f} m, time below "
          f"{100*Pv.vent_h_on*Pv.mac:.0f} cm {100*np.mean(lg['hmin'][w] < Pv.vent_h_on*Pv.mac):4.1f}%, "
          f"heave std {100*lg['z'][w].std():5.2f} cm, pitch std {deg(lg['theta'][w].std()):.2f} deg, failed={lg['failed']}")
    SEA[("vent", tag)] = lg

# ---------------------------------------------------------------- F
print("\nF. actuator rate limit (60 deg/s): step with LQI-SW")
for name in ("reference", "actuator_rate"):
    lg = STEP[(name, "LQI-SW")]
    r = deg(np.diff(lg["act"], axis=0) / 0.005)
    print(f"   {name:15s}: max |rate| {np.abs(r).max():6.1f} deg/s, time at limit {100*np.mean(np.abs(r).max(1) > 59.9):.1f}%")

# ---------------------------------------------------------------- G
print("\nG. roll: linear roll damping and beam seas")
for name, ph in (("reference", Physics()), ("roll_inflow", Physics(roll_inflow=True)),
                 ("roll_inflow+added_mass", Physics(roll_inflow=True, added_mass=True))):
    lin = Hydrofoil(Params(phys=ph), wagner=False)
    tr = TRIMS["reference"]
    A, B = linearize(lin, tr["core"], tr["delta"], tr["rpm"])
    Ixx = Params().Ixx + (Params().MA[1, 1] if ph.added_mass else 0.0)
    print(f"   {name:24s}: dp'/dp = {A[5,5]:7.2f} 1/s  ->  K_p = {A[5,5]*Ixx:7.1f} N m s/rad, "
          f"roll control dp'/d(dFS) = {B[5,0]:7.1f}")
for name, ph in (("reference", Physics()), ("roll_inflow", Physics(roll_inflow=True)), ("v2 (all)", Physics.v2())):
    sup, env = Supervisor(models), Environment(seed=7, aero=True)
    plant = Hydrofoil(Params(phys=ph), wagner=True)
    lg = simulate_supervised(sup, TRIMS[name], T=30.0, model=plant, env=env,
                             events=[(1.0, E({"sea": {"hs": 0.4, "tp": 3.0, "dir": 90, "spread": True}}))])
    w = lg["t"] >= 10
    print(f"   beam sea Hs 0.4, {name:12s}: roll std {deg(lg['phi'][w].std()):.3f} deg, heave std "
          f"{100*lg['z'][w].std():.2f} cm, roll-rate std {deg(lg['p'][w].std()):.2f} deg/s, failed={lg['failed']}")
    SEA[("beam", name)] = lg

# ---------------------------------------------------------------- H
print("\nH. off-design speeds (speed scaling active): 7 -> 5 m/s, then 5 -> 13 m/s")
ev_speed = [(2.0, lambda s: (s.set_accel(0.5), s.set_speed(5.0))), (12.0, lambda s: s.set_speed(13.0))]
for sea in (None, 0.2):
    for name, ph in (("reference", Physics()), ("v2 (all)", Physics.v2())):
        sup, env = Supervisor(models), Environment(seed=11, aero=True)
        plant = Hydrofoil(Params(phys=ph), wagner=True)
        ev = list(ev_speed) + ([(1.0, E({"sea": {"hs": sea, "tp": 2.5, "dir": 0}}))] if sea else [])
        lg = simulate_supervised(sup, TRIMS[name], T=40.0, model=plant, env=env, events=ev)
        tag = f"{'calm' if not sea else 'Hs ' + str(sea)}, {name}"
        parts = []
        for a, b in ((7, 12), (32, 40)):
            w = (lg["t"] >= a) & (lg["t"] <= b)
            if w.any():
                parts.append(f"{lg['u'][w].mean():5.2f} m/s: heave std {100*lg['z'][w].std():5.2f} cm, "
                             f"height err {100*np.abs(lg['z'][w]-lg['zref'][w]).max():5.2f} cm max")
        print(f"   {tag:22s} | " + " | ".join(parts) + f" | failed={lg['failed']}")
        SEA[("speed", tag)] = lg

np.save(ROOT / "figures" / "phase4_logs.npy", dict(step=STEP, sea=SEA), allow_pickle=True)
