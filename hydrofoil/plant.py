"""Nonlinear hydrofoil craft model.

Reference model (Params.phys all False): a block-by-block Python port of
src/implementation_dynamicmodel/hydrofoil_sim_unified.slx (simulation model)
and simplified_sim_for_linearize.slx (linearisation model, no Wagner states,
no actuator lag) from jskoun/MPC_HydrofoilControl.

Phase 4 upgrades (Params.phys, each independently switchable):
  added_mass     foil/strut added mass couples heave, roll and pitch accelerations
  speed_scaling  unsteady-lift poles and wave-drag Froude number follow the actual speed
  lift_lag_only  only circulatory lift is lagged; wave gusts use Kussner's function
  roll_inflow    roll rate enters the foil inflow, roll damping from strip theory
  ventilation    lift collapse near the free surface with hysteresis (assumed numbers)
  downwash       fore-foil wakes induce flow at the aft foil (horseshoe vortices)
  actuator_rate  servo rate limit

Reduced DOF exactly as thesis Sec. 3.1: sway, yaw, yaw-rate locked at zero.
Core state (8): [xN, z, phi, theta, u, w, p, q]   (z NED, down positive)
Full state (28): core | 3 actuator positions | Wagner 3x2 | Kussner 3x2 |
                 ventilation 3 | lagged fore-foil circulation 2
(states of a switched-off feature simply stay constant).

Deviation from the Simulink source in the reference model (numerics only):
 * if a foil leaves the water (h_sub <= 0) its lift/drag/moment are set to 0.
   The original formulas are undefined there (sqrt of negative h, Ldec < 0).
"""
from dataclasses import dataclass

import numpy as np

from .params import Params
from .wake import WakeTable
from .waves import calm

CORE = ["xN", "z", "phi", "theta", "u", "w", "p", "q"]
# state slices
S_ACT, S_WAG, S_KUS, S_VENT, S_GAM = slice(8, 11), slice(11, 17), slice(17, 23), slice(23, 26), slice(26, 28)
NX = 28
# Kussner function (Sears and Sparks): psi(s) = 1 - 0.5 exp(-0.13 s) - 0.5 exp(-s),
# s = distance travelled in semi-chords
KUS_B1, KUS_B2 = 0.13, 1.0
WAG_B1, WAG_B2 = 0.0455, 0.3


@dataclass
class Env:
    """Environment seen by the force model (phase 3; not part of the reference
    model, which has still water of fixed density and no air forces).
    Vectors are horizontal NED [north, east] in m/s."""
    rho: float | None = None            # water density; None = Params.rho
    current: tuple = (0.0, 0.0)         # water velocity over ground
    wind: tuple = (0.0, 0.0)            # air velocity over ground
    aero: bool = False                  # apply air drag on the hull (off = reference model)
    rho_air: float = 1.225


@dataclass
class Unsteady:
    """Internal (filter) states handed to forces().  None = quasi-steady."""
    wag: np.ndarray        # (3, 2) Wagner
    kus: np.ndarray        # (3, 2) Kussner
    vent: np.ndarray       # (3,)   ventilation fraction 0..1
    gam: np.ndarray        # (2,)   lagged circulation of the fore foils


def _interp(x, xp, fp):
    """Linear interpolation with linear extrapolation (Simulink n-D Lookup
    default 'Linear point-slope')."""
    if x <= xp[0]:
        return fp[0] + (fp[1] - fp[0]) / (xp[1] - xp[0]) * (x - xp[0])
    if x >= xp[-1]:
        return fp[-1] + (fp[-1] - fp[-2]) / (xp[-1] - xp[-2]) * (x - xp[-1])
    return np.interp(x, xp, fp)


def _slope(x, xp, fp):
    """d fp / d xp of the piecewise-linear table at x."""
    i = int(np.clip(np.searchsorted(xp, x) - 1, 0, len(xp) - 2))
    return (fp[i + 1] - fp[i]) / (xp[i + 1] - xp[i])


def dcm(phi, theta):
    """R^n_b for psi = 0 (thesis eq. 2.2.6)."""
    cf, sf, ct, st = np.cos(phi), np.sin(phi), np.cos(theta), np.sin(theta)
    return np.array([[ct, sf * st, cf * st],
                     [0.0, cf, -sf],
                     [-st, sf * ct, cf * ct]])


class Hydrofoil:
    def __init__(self, params: Params | None = None, wagner=True):
        self.P = params or Params()
        self.wagner = wagner
        self.foils = self.P.foils()
        self.nx = NX
        self.env = Env()
        self.wake = WakeTable(self.P.arm_FS, self.P.arm_A, self.P.span)   # cheap; physics can change in flight
        self._last = {}

    # ------------------------------------------------------------------ forces
    def forces(self, core, alpha_eff_fn, delta, rpm, t, waves, out=None, ust: Unsteady | None = None):
        """Generalised force [X, Z, K, M] on the craft.

        Reference path: alpha_eff_fn(i, alpha) maps the quasi-steady hydrodynamic
        angle of foil i to the angle used in the polar lookups (identity, or the
        Wagner output).  If `ust` is given (full simulation) the unsteady states
        are taken from it instead and alpha_eff_fn is ignored.
        Returns (tau, alphas); alphas are the quasi-steady angles.  Quantities
        needed for the filter derivatives are left in self._last."""
        P, F_, E = self.P, self.P.phys, self.env
        rho = P.rho if E.rho is None else E.rho
        xN, z, phi, th, u, w, p, q = core
        R = dcm(phi, th)
        cur_n = np.array([E.current[0], E.current[1], 0.0])
        cur_b = R.T @ cur_n
        u_w = u - cur_b[0]                  # forward speed through the water
        X = Z = K = M = 0.0
        alphas = np.zeros(3)
        a_mot, a_wav, Vf, hf, gam_now = np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(2)
        pol = P.polar
        bprime = np.pi / 4 * P.span
        w_up = 0.0
        for i, (name, r, S, cs, Ld) in enumerate(self.foils):
            lx, ly, lz = r
            pos = np.array([xN, 0.0, z]) + R @ r
            zeta, vw_n = waves.at(pos[0], pos[1], pos[2], t)
            h = pos[2] + zeta                       # submergence (wave_elev_sub)
            # pivot velocity: v + omega x r.  The Simulink model leaves ly out
            # (Mux2 input 2 unconnected) and adds roll damping separately.
            v_piv = np.array([u + q * lz, -p * lz, w - q * lx + (p * ly if F_.roll_inflow else 0.0)])
            if i == 2 and F_.downwash:
                g = ust.gam if ust is not None else gam_now
                w_up = self.wake(hf[0]) * g[0] + self.wake(hf[1]) * g[1]
                v_piv[2] += w_up                    # upwash = water moving up = larger incidence
            v_mot = v_piv - cur_b                   # relative velocity without waves
            v_rel = v_piv - R.T @ (vw_n + cur_n)    # 'Sum' |+- in Foil Theory
            a_inf = np.arctan(v_rel[2] / v_rel[0])
            V2 = v_rel[0] ** 2 + v_rel[2] ** 2
            V = np.sqrt(V2)
            alphas[i] = a_inf + delta[i]
            a_m = np.arctan(v_mot[2] / v_mot[0]) + delta[i]
            a_mot[i], a_wav[i], Vf[i], hf[i] = a_m, alphas[i] - a_m, V, h
            # ---- effective angles
            if ust is None:
                a_circ = a_visc = alpha_eff_fn(i, alphas[i]) if alpha_eff_fn else alphas[i]
            elif F_.lift_lag_only:
                a_circ = 0.5 * a_m + ust.wag[i].sum() + ust.kus[i].sum()   # Wagner(motion) + Kussner(waves)
                a_visc = alphas[i]
            else:
                a_circ = a_visc = (0.5 * alphas[i] + ust.wag[i].sum()) if self.wagner else alphas[i]
            ac_deg, av_deg = np.rad2deg(a_circ), np.rad2deg(a_visc)
            # ---- roll damping
            if F_.roll_inflow:
                # ly is in the inflow; what remains is the foil's own span, by strip
                # theory with the local lift slope of the polar (int (y-ly)^2 c dy = S b^2 / 16)
                cla = np.rad2deg(_slope(ac_deg, pol["alpha"], pol["cl"]))
                K += -0.5 * rho * cla * (S * P.span ** 2 / 16) * V * p
            else:
                K += -0.5 * rho * 2 * np.pi * Ld * (v_piv[0] - cur_b[0]) * p
            if h <= 0.0:          # out of the water: see module docstring
                continue
            cl = _interp(ac_deg, pol["alpha"], pol["cl"])
            cdi = _interp(ac_deg, pol["alpha"], pol["cdi"])
            cdv = _interp(av_deg, pol["alpha"], pol["cdv"])
            cm = _interp(av_deg, pol["alpha"], pol["cm"])
            if F_.ventilation:
                v = (ust.vent[i] if ust is not None else float(h < P.vent_h_on * P.mac))
                kv = 1.0 - (1.0 - P.vent_lift_factor) * v
                cl, cdi, cm = cl * kv, cdi * kv * kv, cm * kv
            ldec = 1 - 0.422 * np.exp(-1.454 * h / P.mac)            # (2.5.17)
            hs = abs(h)
            x15 = 33 * (hs / P.span) ** 1.5
            dinc = (2 - x15 / (x15 + 1)) * cdi                        # (2.5.18)
            frh = (V if F_.speed_scaling else P.usted) / np.sqrt(P.g * hs)
            cdw = 1 / (2 * frh ** 2) * (P.mac / hs) * np.exp(-2 / frh ** 2)
            tc = P.t_foil / P.mac
            cdint = 0.75 * tc - 0.0003 * tc ** -2                     # (2.5.39)
            qS = 0.5 * rho * S * V2
            L = qS * cl * ldec
            D = qS * (cdv + dinc) + qS * cl ** 2 * cdw + 0.5 * rho * P.t_foil ** 2 * V2 * cdint
            Mf = qS * P.mac * cm * ldec
            if i < 2:
                gam_now[i] = L / (rho * V * bprime)                   # circulation of the horseshoe
            ca, sa = np.cos(a_inf), np.sin(a_inf)
            F = np.array([ca * -D - sa * -L, 0.0, sa * -D + ca * -L])  # Rbh*[-D,0,-L]
            mom = np.cross(r, F)
            # strut (STRUT block): drag, its pitch moment, and roll damping
            Xs = -0.5 * rho * cs * u_w ** 2 * P.cd_strut0 * h
            Ms = Xs * (lz - 0.5 * h)
            if F_.roll_inflow:    # wetted strut from the waterline (lz - h) down to the foil
                Ks = -0.5 * rho * u_w * P.strut_cla * p * cs * (lz ** 3 - (lz - h) ** 3) / 3
            else:                 # reference formula, including its unexplained 0.1 m offset
                Ks = -0.5 * rho * u_w * 2 * np.pi * p * cs * (lz ** 3 - (lz + 0.1 - h) ** 3) / 3
            X += F[0] + Xs
            Z += F[2]
            K += mom[0] + Ks
            M += mom[1] + Mf + Ms
            if out is not None:
                out[name] = dict(h=h, L=L, D=D, alpha_deg=ac_deg, V2=V2)
        # gravity (Gravity block, r_g = 0)
        X += -P.mass * P.g * np.sin(th)
        Z += P.mass * P.g * np.cos(th) * np.cos(phi)
        # propeller (MSS open-water block, quadratic KT fit)
        n = rpm / 60.0
        if abs(n) > 1e-6:
            J = (1 - P.prop_wake) * u_w / (n * P.prop_D)
            T = rho * n * abs(n) * P.prop_D ** 4 * np.polyval(P.KT_poly, J)
        else:
            T = 0.0
        X += T
        M += P.prop_arm_z * T
        # air drag on the hull / deck (phase 3, ASSUMED coefficients in Params)
        if E.aero:
            wind_b = R.T @ np.array([E.wind[0], E.wind[1], 0.0])
            vax, vay = u - wind_b[0], -wind_b[1]          # craft velocity through the air (v = 0)
            Fax = -0.5 * E.rho_air * P.air_CdA_x * vax * abs(vax)
            Fay = -0.5 * E.rho_air * P.air_CdA_y * vay * abs(vay)
            X += Fax
            K += P.air_z_ce * Fay          # force applied z_ce above the CG
            M += -P.air_z_ce * Fax
            if out is not None:
                out["aero"] = (Fax, Fay)
        if out is not None:
            out["thrust"] = T
            out["upwash"] = w_up
        self._last = dict(a_mot=a_mot, a_wav=a_wav, V=Vf, h=hf, gam=gam_now)
        return np.array([X, Z, K, M]), alphas

    # ------------------------------------------------------------ derivatives
    def core_deriv(self, core, tau):
        P = self.P
        xN, z, phi, th, u, w, p, q = core
        X, Z, K, M = tau
        if P.phys.added_mass:
            # (M_RB + M_A) [w', p', q'] = [Z + m q u, K, M]
            Mm = np.diag([P.mass, P.Ixx, P.Iyy]) + P.MA
            wd, pd, qd = np.linalg.solve(Mm, [Z + P.mass * q * u, K, M])
        else:
            wd, pd, qd = Z / P.mass + q * u, K / P.Ixx, M / P.Iyy
        return np.array([
            u * np.cos(th) + w * np.cos(phi) * np.sin(th),       # xN dot
            -u * np.sin(th) + w * np.cos(th) * np.cos(phi),      # z dot
            p + q * np.sin(phi) * np.tan(th),                    # phi dot
            q * np.cos(phi),                                     # theta dot
            X / P.mass - q * w,                                  # u dot
            wd, pd, qd,
        ])

    def deriv_simplified(self, core, delta, rpm, t=0.0, waves=None):
        """simplified_sim_for_linearize: deflections enter directly, quasi-
        steady lift (no unsteady-lift states, no actuator dynamics)."""
        tau, _ = self.forces(core, lambda i, a: a, delta, rpm, t, waves or calm())
        return self.core_deriv(core, tau)

    def deriv(self, x, delta_cmd, rpm, t, waves):
        """Full simulation model: actuators + unsteady states."""
        P, F_ = self.P, self.P.phys
        core, act = x[:8], x[S_ACT]
        ust = Unsteady(x[S_WAG].reshape(3, 2), x[S_KUS].reshape(3, 2), x[S_VENT], x[S_GAM])
        tau, alphas = self.forces(core, None, act, rpm, t, waves, ust=ust)
        L = self._last
        dx = np.zeros_like(x)
        # actuators
        dact = (np.asarray(delta_cmd) - act) / P.act_tau
        if F_.actuator_rate:
            dact = np.clip(dact, -P.act_rate, P.act_rate)
        dx[S_ACT] = dact
        if F_.added_mass:
            # non-circulatory reaction to the foil rotating in the stream:
            # normal velocity of the chord ~ w_foil + U delta  ->  Fz = -A33 U delta_dot
            for i, (name, r, S, cs, Ld) in enumerate(self.foils):
                Fz = -P.A33[i] * L["V"][i] * dact[i]
                tau = tau + np.array([0.0, Fz, r[1] * Fz, -r[0] * Fz])
        dx[:8] = self.core_deriv(core, tau)
        # unsteady lift
        half_c = P.mac / 2
        if self.wagner:
            Vs = L["V"] if F_.speed_scaling else np.full(3, P.usted)
            drive = L["a_mot"] if F_.lift_lag_only else alphas
            wag = ust.wag
            d = np.empty((3, 2))
            d[:, 0] = WAG_B1 * Vs / half_c * (P.wagC1 * drive - wag[:, 0])
            d[:, 1] = WAG_B2 * Vs / half_c * (P.wagC2 * drive - wag[:, 1])
            dx[S_WAG] = d.ravel()
            if F_.lift_lag_only:
                kus = ust.kus
                d = np.empty((3, 2))
                d[:, 0] = KUS_B1 * Vs / half_c * (0.5 * L["a_wav"] - kus[:, 0])
                d[:, 1] = KUS_B2 * Vs / half_c * (0.5 * L["a_wav"] - kus[:, 1])
                dx[S_KUS] = d.ravel()
        if F_.ventilation:
            h, v = L["h"], ust.vent
            on, off = h < P.vent_h_on * P.mac, h > P.vent_h_off * P.mac
            dx[S_VENT] = np.where(on, (1 - v) / P.vent_tau_on, np.where(off, -v / P.vent_tau_off, 0.0))
        if F_.downwash:
            # the wake needs (x_fore - x_aft) / V to reach the aft foil: first-order lag
            tau_w = (P.arm_FS[0] - P.arm_A[0]) / np.maximum(L["V"][:2], 1.0)
            dx[S_GAM] = (L["gam"] - ust.gam) / tau_w
        return dx

    def set_physics(self, phys, x):
        """Change the physics switches in flight: filters re-initialised at the
        current state, actuators kept (mirror of libfoil Craft::setPhysics)."""
        self.P.phys = phys
        x2 = self.initial_state(x[:8], x[S_ACT])
        x2[:8], x2[S_ACT] = x[:8], x[S_ACT]
        return x2

    def initial_state(self, core, delta):
        """Full state at rest on a trim point (all filters at steady state)."""
        P = self.P
        x = np.zeros(self.nx)
        x[:8] = core
        x[S_ACT] = delta
        _, alphas = self.forces(core, lambda i, a: a, delta, 0.0, 0.0, calm())
        L = self._last
        drive = L["a_mot"] if P.phys.lift_lag_only else alphas
        if self.wagner:
            x[S_WAG] = np.column_stack([P.wagC1 * drive, P.wagC2 * drive]).ravel()
            x[S_KUS] = np.column_stack([0.5 * L["a_wav"], 0.5 * L["a_wav"]]).ravel()
        x[S_GAM] = L["gam"]
        return x


def rk4(f, x, dt):
    k1 = f(x)
    k2 = f(x + 0.5 * dt * k1)
    k3 = f(x + 0.5 * dt * k2)
    k4 = f(x + dt * k3)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
