"""Closed-loop simulation harness (stands in for hydrofoil_sim_unified.slx +
benchmark.m).

Timing as in the Simulink model: plant integrated with fixed-step RK4 at
5 ms (Simulink used a variable-step solver), foil controller at 10 Hz with
zero-order hold, speed PI on rpm at 100 Hz (P = 250, I = 200, around 1200 rpm),
first-order actuator lag tau = 0.1 s, Wagner unsteady lift on all foils.
Full state feedback (the repo's sensorfeedback_flag = 0 case).
"""
import numpy as np

from .plant import Hydrofoil, rk4
from .waves import calm

DT = 0.005
CTRL_EVERY = 20       # 0.1 s
PI_EVERY = 2          # 0.01 s


def smoothstep(t, t0, dur, inc):
    tau = np.clip((t - t0) / dur, 0, 1)
    return inc * (3 * tau ** 2 - 2 * tau ** 3)


def make_reference(kind="step", z0=-0.3, dz=-0.2, t0=0.5, dur=0.5):
    """Returns r(t) -> 6-vector [z, phi, theta, w, p, q] reference.
    dz = -0.2 is a +0.2 m rise in ride height (z is NED, down positive)."""
    def r(t):
        out = np.zeros(6)
        if kind == "step":
            out[0] = z0 + (dz if t >= t0 else 0.0)
        elif kind == "ramp":
            out[0] = z0 + dz * np.clip((t - t0) / dur, 0, 1)
        elif kind == "smoothramp":
            out[0] = z0 + smoothstep(t, t0, dur, dz)
        else:
            out[0] = z0
        return out
    return r


def simulate(controller, trim, T=4.0, ref=None, u_ref=7.0, waves=None, model=None,
             x0_offset=None, params=None, speed_ref=None):
    model = model or Hydrofoil(params=params, wagner=True)
    waves = waves or calm()
    ref = ref or make_reference("none")
    core0, delta0, rpm0 = trim["core"], trim["delta"], trim["rpm"]
    x = model.initial_state(core0, delta0)
    if x0_offset is not None:
        x[:8] += x0_offset
    P = model.P
    xi = (rpm0 - P.rpm_trim_ff) / P.pi_I       # PI integrator preset at trim rpm
    rpm = rpm0
    u_cmd = delta0.copy()
    n = int(round(T / DT))
    log = {k: np.zeros(n + 1) for k in ["t", "z", "phi", "theta", "u", "w", "p", "q", "rpm", "zref",
                                         "hFS", "hFP", "hA"]}
    log["act"] = np.zeros((n + 1, 3)); log["cmd"] = np.zeros((n + 1, 3))
    controller.reset()
    broached = False
    for k in range(n + 1):
        t = k * DT
        if k % CTRL_EVERY == 0:
            y = x[[1, 2, 3, 5, 6, 7]]
            # preview rows start at the current sample (MPC Toolbox / mpcblock_preview
            # convention); this alignment reproduces thesis Table 5.5 for MPC-IDP
            rf = [ref(t + TS_i) for TS_i in np.arange(0, 10) * 0.1]
            u_cmd = np.asarray(controller(y, ref(t), rf, speed=x[4]), float)
        if k % PI_EVERY == 0:
            ur = speed_ref(t) if speed_ref else u_ref
            e = ur - x[4]
            rpm = P.rpm_trim_ff + P.pi_P * e + P.pi_I * xi
            xi += e * P.pi_Ts
        out = {}
        model.forces(x[:8], lambda i, a: a, x[8:11], rpm, t, waves, out=out)
        log["t"][k] = t
        for j, name in enumerate(["z", "phi", "theta", "u", "w", "p", "q"]):
            log[name][k] = x[1 + j]
        log["rpm"][k] = rpm; log["zref"][k] = ref(t)[0]
        log["act"][k] = x[8:11]; log["cmd"][k] = u_cmd
        for nm in ("FS", "FP", "A"):
            log["h" + nm][k] = out.get(nm, {}).get("h", np.nan)
        if not np.all(np.isfinite(x)) or x[1] > 0.6 or x[1] < -1.5:
            broached = True
            for key in log:
                log[key] = log[key][:k + 1]
            break
        if k < n:
            x = rk4(lambda s: model.deriv(s, u_cmd, rpm, t, waves), x, DT)
    log["failed"] = broached
    return log


def metrics(log, trim_delta, t_from=0.0):
    """benchmark.m evaluatePerformance(): scaled z/0.1 m, phi/5 deg,
    theta/5 deg errors; input = actuator position minus trim, scaled /5 deg."""
    t = log["t"]
    m = t >= t_from
    t = t[m]
    e = np.column_stack([(log["z"][m] - log["zref"][m]) / 0.1,
                         log["phi"][m] / np.deg2rad(5), log["theta"][m] / np.deg2rad(5)])
    u = (log["act"][m] - trim_delta) / np.deg2rad(5)
    return dict(ISE=np.trapezoid(np.sum(e ** 2, 1), t),
                IAE=np.trapezoid(np.sum(np.abs(e), 1), t),
                ITAE=np.trapezoid(t * np.sum(np.abs(e), 1), t),
                effort=np.trapezoid(np.sum(u ** 2, 1), t),
                failed=bool(log["failed"]))


def simulate_supervised(sup, trim, T=10.0, events=(), waves=None, model=None, params=None, env=None,
                        cmd_delay_steps=0):
    """Closed loop with a Supervisor (hydrofoil/supervisor.py): the MPC commands
    the three foils AND propeller rpm at 10 Hz (no separate speed PI).
    events: iterable of (time, callable) operator actions; the callable takes
    (sup) or (sup, env, t).
    env: optional hydrofoil.environment.Environment (sea, wind, current, water);
    it is stepped at 10 Hz and overrides `waves`.
    cmd_delay_steps: commands take effect this many 5 ms plant steps after the
    measurement they were computed from (models computation/transport delay)."""
    import inspect
    model = model or Hydrofoil(params=params, wagner=True)
    waves = waves or calm()
    if env is not None:
        waves = env.sea_state
        env.step(0.0)
        model.env = env.plant_env()
    x = model.initial_state(trim["core"], trim["delta"])
    rpm = float(trim["rpm"])
    u_cmd = trim["delta"].copy()
    events = sorted(events, key=lambda e: e[0])
    ei = 0
    n = int(round(T / DT))
    keys = ["t", "z", "phi", "theta", "u", "w", "p", "q", "rpm", "zref", "uref", "hmin", "accel"]
    log = {k: np.zeros(n + 1) for k in keys}
    log["act"] = np.zeros((n + 1, 3)); log["cmd"] = np.zeros((n + 1, 3))
    log["mode"] = []; log["failsafe"] = []
    failed = False
    u_last = x[4]
    pending = None
    for k in range(n + 1):
        t = k * DT
        out = {}
        model.forces(x[:8], lambda i, a: a, x[8:11], rpm, t, waves, out=out)
        h = np.array([out.get(nm, {}).get("h", 0.0) for nm in ("FS", "FP", "A")])
        if k % CTRL_EVERY == 0:
            while ei < len(events) and events[ei][0] <= t + 1e-9:
                fn = events[ei][1]; ei += 1
                fn(sup, env, t) if len(inspect.signature(fn).parameters) >= 3 else fn(sup)
            if env is not None:
                env.step(t)
                model.env = env.plant_env()
                if env.physics is not None and env.physics != model.P.phys:
                    x = model.set_physics(env.physics, x)
            y = x[[1, 2, 3, 4, 5, 6, 7]]
            c = sup.step(t, y, h)
            pending = (k + cmd_delay_steps, c[:3].copy(), float(c[3]))
            log["mode"].append((t, sup.mode)); log["failsafe"].append((t, sup.failsafe))
        if pending is not None and k >= pending[0]:     # computation / transport delay
            u_cmd, rpm = pending[1], pending[2]
            pending = None
        log["t"][k] = t
        for j, name in enumerate(["z", "phi", "theta", "u", "w", "p", "q"]):
            log[name][k] = x[1 + j]
        log["rpm"][k] = rpm; log["zref"][k] = -sup.guid.h_ref; log["uref"][k] = sup.guid.u_ref
        log["hmin"][k] = h.min(); log["act"][k] = x[8:11]; log["cmd"][k] = u_cmd
        log["accel"][k] = (x[4] - u_last) / DT if k else 0.0
        u_last = x[4]
        if not np.all(np.isfinite(x)) or x[1] > 0.6 or x[1] < -1.5:
            failed = True
            for key in keys + ["act", "cmd"]:
                log[key] = log[key][:k + 1]
            break
        if k < n:
            x = rk4(lambda s: model.deriv(s, u_cmd, rpm, t, waves), x, DT)
    log["failed"] = failed
    return log
