"""Controllers ported from jskoun/MPC_HydrofoilControl.

LQISW  - calculate_lqr.m + the 'Discrete LQRi' MATLAB Function block (chart_164)
MPC    - calculate_mpc.m / generate_mpc.m re-implemented without the MATLAB MPC
         Toolbox: same horizons, weights, scale factors, constraints, nominal
         offsets and disturbance-model gains.  The Toolbox's built-in state
         estimator is replaced by a steady-state Kalman filter on the same
         augmented model (see MPC docstring) -- this is an approximation.

All controllers act on the 6 measured states y = [z, phi, theta, w, p, q]
and return absolute foil deflection commands [d_FS, d_FP, d_A] in rad.
"""
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import expm, solve_discrete_are

MAXD = np.deg2rad(10.0)
TS = 0.1
Y_NOM = np.array([-0.3, 0, 0, 0, 0, 0])


def c2d(A, B, Ts):
    n, m = B.shape
    M = np.zeros((n + m, n + m))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M * Ts)
    return E[:n, :n], E[:n, n:]


def dlqr(A, B, Q, R):
    P = solve_discrete_are(A, B, Q, R)
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


# --------------------------------------------------------------------- LQI-SW
class LQISW:
    """Discrete LQR with integral action on [z, phi, theta], saturation and
    anti-windup.  Gains exactly as calculate_lqr.m:
      Q = diag(([0.1 md md .5 .5 .5 .1 md md].^-1 .* [.2 .15 .25 .2 .2 .2 .2 .2 .4]).^2)
      R = diag(([md md md].^-1 .* [.2 .2 .2]).^2),  md = 10 deg
      Klqid = dlqr(c2d(augmented continuous model, 0.1), Q, R)
      Ka = -2 * pinv(Klqid(:, 7:9))
    (Table 5.1 of the thesis lists the same Q, R.)"""
    name = "LQI-SW"

    def __init__(self, Ar, Br, u_trim, saturate=True, antiwindup=True):
        md = MAXD
        q = (np.array([0.1, md, md, .5, .5, .5, .1, md, md]) ** -1 *
             np.array([.2, .15, .25, .2, .2, .2, .2, .2, .4])) ** 2
        r = (np.array([md, md, md]) ** -1 * np.array([.2, .2, .2])) ** 2
        Aaug = np.block([[Ar, np.zeros((6, 3))], [np.eye(3), np.zeros((3, 6))]])
        Baug = np.vstack([Br, np.zeros((3, 3))])
        Ad, Bd = c2d(Aaug, Baug, TS)
        self.K = dlqr(Ad, Bd, np.diag(q), np.diag(r))
        self.Ka = -np.linalg.pinv(self.K[:, 6:9]) * 2
        self.u_trim = np.asarray(u_trim)
        self.sat, self.aw = saturate, antiwindup
        self.reset()

    def reset(self):
        self.xI = np.zeros(3)
        self.u_c = np.zeros(3)
        self.u_hold = np.zeros(3)

    def __call__(self, y, ref, ref_future=None, speed=None):
        e = y - ref
        aw = self.Ka @ (self.u_hold - self.u_c) if self.aw else 0.0
        self.xI = self.xI + TS * e[:3] + TS * aw
        self.u_c = self.u_trim - self.K @ np.r_[e, self.xI]
        self.u_hold = np.clip(self.u_c, -MAXD, MAXD) if self.sat else self.u_c.copy()
        return self.u_hold


# ------------------------------------------------------------------------ MPC
class MPC:
    """Linear (or LPV) MPC, Toolbox-equivalent formulation.

    variant flags (thesis Table 4.3):
      I  input-disturbance integrators (always on here; thesis final designs use it)
      D  actuator first-order lag (tau = 0.1 s) appended to the prediction model
      P  reference preview (future reference over the horizon)
      lpv  prediction model + nominal offsets scheduled on forward speed u

    Cost (MPC Toolbox standard form, weights act on scaled signals):
      sum_{i=1..Hp} |Wy (y_i - r_i)/Ly|^2 + sum_{i=0..Hp-1} |Wu (u_i - u_nom)/Lu|^2
      + sum_{i=0..Hu-1} |Wdu du_i / Lu|^2
    Constraints: |u| <= 10 deg (absolute), du <= +3 deg per step (generate_mpc.m
    sets only RateMax, so RateMin stays -Inf as in the Toolbox default).

    Estimator (APPROXIMATION of the Toolbox default): steady-state Kalman filter,
    'current' form, on [plant (+actuator) ; input-disturbance integrators].
    Disturbance integrators driven by white noise with gain
    gd * Ts (Toolbox default integrator gain Ts, scaled by 0.4 for -I/-IP and 1.5
    for -ID/-IDP exactly as calculate_mpc.m does), measurement noise unit
    variance in scaled output units, tiny process noise on plant states.
    """

    def __init__(self, models, nominal_idx, delay=False, preview=False, lpv=False,
                 Hp=10, Hu=2, Wy=(4, 3, 4, .1, .1, .1), Wu=(.2, .2, .2),
                 Wdu=(.15, .15, .2), Ly=(0.2, MAXD, MAXD, .5, .5, .5), Lu=MAXD,
                 dist_gain=None, tau=0.1, rate_max=np.deg2rad(3)):
        self.delay, self.preview, self.lpv = delay, preview, lpv
        self.name = "LPV-" * lpv + "MPC-I" + "D" * delay + "P" * preview
        self.Hp, self.Hu = Hp, Hu
        self.wy = np.array(Wy) / np.array(Ly)
        self.wu = np.array(Wu) / Lu
        self.wdu = np.array(Wdu) / Lu
        self.Ly = np.array(Ly)
        self.rate_max = rate_max
        gd = dist_gain if dist_gain is not None else (1.5 if delay else 0.4)
        self.speeds = np.array([m["U"] for m in models])
        self.grid = [self._build(m, tau, gd) for m in models]
        self.nom = nominal_idx
        self.reset()

    # -- model construction --------------------------------------------------
    def _build(self, m, tau, gd):
        A, B = m["Ar"], m["Br"]
        if self.delay:
            Ac = np.block([[A, B], [np.zeros((3, 6)), -np.eye(3) / tau]])
            Bc = np.vstack([np.zeros((6, 3)), np.eye(3) / tau])
        else:
            Ac, Bc = A, B
        Ad, Bd = c2d(Ac, Bc, TS)
        npl = Ad.shape[0]
        # augmented: [xp ; d], d adds to the MV (unmeasured input disturbance)
        Aa = np.block([[Ad, Bd], [np.zeros((3, npl)), np.eye(3)]])
        Ba = np.vstack([Bd, np.zeros((3, 3))])
        Ca = np.hstack([np.eye(6), np.zeros((6, npl - 6 + 3))])
        G = np.vstack([np.zeros((npl, 3)), gd * TS * np.eye(3)])
        Qn = G @ G.T + 1e-8 * np.eye(npl + 3)
        Rn = np.diag(self.Ly ** 2)
        P = solve_discrete_are(Aa.T, Ca.T, Qn, Rn)
        M = P @ Ca.T @ np.linalg.inv(Ca @ P @ Ca.T + Rn)     # current-form gain
        return dict(Aa=Aa, Ba=Ba, Ca=Ca, M=M, u_nom=m["delta"].copy(),
                    nx=npl + 3, npl=npl)

    def _model_at(self, speed):
        if not self.lpv or speed is None:
            return self.grid[self.nom]
        s = np.clip(speed, self.speeds[0], self.speeds[-1])
        j = int(np.clip(np.searchsorted(self.speeds, s) - 1, 0, len(self.speeds) - 2))
        a = (s - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
        g0, g1 = self.grid[j], self.grid[j + 1]
        return {k: (1 - a) * g0[k] + a * g1[k] if isinstance(g0[k], np.ndarray) else g0[k]
                for k in g0}

    def reset(self):
        g = self.grid[self.nom]
        self.xhat = np.zeros(g["nx"])          # deviation from nominal
        self.u_prev = np.zeros(3)              # deviation command applied last step
        self.solver = None

    # -- one control step ----------------------------------------------------
    def __call__(self, y, ref, ref_future=None, speed=None):
        g = self._model_at(speed)
        Aa, Ba, Ca, M = g["Aa"], g["Ba"], g["Ca"], g["M"]
        u_nom = g["u_nom"]
        # if the schedule moved the nominal input, keep the applied absolute command
        if not hasattr(self, "_u_nom_last"):
            self._u_nom_last = u_nom
        self.u_prev = self.u_prev + self._u_nom_last - u_nom
        self._u_nom_last = u_nom
        # estimator: time update (done at end of last call) + measurement update
        dy = y - Y_NOM
        self.xhat = self.xhat + M @ (dy - Ca @ self.xhat)
        Hp, Hu = self.Hp, self.Hu
        if self.preview and ref_future is not None:
            R = np.array(ref_future[:Hp]) - Y_NOM
        else:
            R = np.tile(ref - Y_NOM, (Hp, 1))
        # prediction: y_i = Ca (Aa^i x0 + sum_j Aa^(i-1-j) Ba u_j), u_j = u_prev + S_j dU
        nx = Aa.shape[0]
        S = np.zeros((Hp, 3, 3 * Hu))          # u_j = u_prev + S[j] @ dU
        for j in range(Hp):
            for l in range(min(j, Hu - 1) + 1):
                S[j][:, 3 * l:3 * l + 3] = np.eye(3)
        Apow = [np.eye(nx)]
        for _ in range(Hp):
            Apow.append(Aa @ Apow[-1])
        rows_H, rows_f = [], []
        for i in range(1, Hp + 1):
            free = Ca @ Apow[i] @ self.xhat
            forced_const = np.zeros(6)
            forced_dU = np.zeros((6, 3 * Hu))
            for j in range(i):
                CAB = Ca @ Apow[i - 1 - j] @ Ba
                forced_const += CAB @ self.u_prev
                forced_dU += CAB @ S[j]
            W = np.diag(self.wy)
            rows_H.append(W @ forced_dU)
            rows_f.append(W @ (free + forced_const - R[i - 1]))
        for j in range(Hp):                   # MV-to-nominal tracking
            W = np.diag(self.wu)
            rows_H.append(W @ S[j]); rows_f.append(W @ self.u_prev)
        for l in range(Hu):                   # move suppression
            E = np.zeros((3, 3 * Hu)); E[:, 3 * l:3 * l + 3] = np.diag(self.wdu)
            rows_H.append(E); rows_f.append(np.zeros(3))
        Hm, fm = np.vstack(rows_H), np.concatenate(rows_f)
        Pq = Hm.T @ Hm
        qv = Hm.T @ fm
        # constraints on absolute u for the Hu distinct moves, and du <= rate_max
        nU = 3 * Hu
        Acons, lo, hi = [], [], []
        for l in range(Hu):
            Acons.append(S[l]); lo.append(-MAXD - u_nom - self.u_prev); hi.append(MAXD - u_nom - self.u_prev)
        Acons.append(np.eye(nU)); lo.append(-np.inf * np.ones(nU)); hi.append(self.rate_max * np.ones(nU))
        Ac = sparse.csc_matrix(np.vstack(Acons))
        lo, hi = np.concatenate(lo), np.concatenate(hi)
        Pc = sparse.csc_matrix(np.triu(Pq + 1e-9 * np.eye(nU)))
        # 6 decision variables: a fresh OSQP setup per step is cheap and avoids
        # sparsity-pattern changes between updates
        self.solver = osqp.OSQP()
        self.solver.setup(Pc, qv, Ac, lo, hi, verbose=False, eps_abs=1e-8, eps_rel=1e-8,
                          polish=False, max_iter=20000)
        res = self.solver.solve()
        dU = res.x if res.x is not None and np.all(np.isfinite(res.x)) else np.zeros(nU)
        u_dev = self.u_prev + dU[:3]
        u_abs = np.clip(u_nom + u_dev, -MAXD, MAXD)
        u_dev = u_abs - u_nom
        # estimator time update with the applied input
        self.xhat = Aa @ self.xhat + Ba @ u_dev
        self.u_prev = u_dev
        return u_abs
