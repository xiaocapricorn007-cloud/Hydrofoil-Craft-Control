"""Speed + height MPC (phase 2): one controller for ride height, roll, pitch AND
forward speed, with operator foil overrides.

This is OUR design, not a port.  It extends the reference MPC-IDP idea
(input-disturbance integrators + actuator-lag model + reference preview) to

  outputs  y = [z, phi, theta, u, w, p, q]          (7, all measured)
  inputs   v = [d_FS, d_FP, d_A, rpm]               (4)

and schedules the prediction model and its trim point on the measured speed
(21 linearisations, 5-15 m/s).  Because the nominal input is the trim for the
*current* speed, the "stay near nominal" input penalty vanishes at any steady
speed, so it causes no steady-state offset.

Foil overrides: an overridden foil is fixed in the QP by equal lower/upper
bounds, so the optimiser knows about it and re-plans the remaining inputs.
With 3 foils and 3 attitude/height outputs there is no redundancy: each locked
foil removes one degree of freedom, and the weights decide what is given up.
"""
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import solve_discrete_are

from .controllers import c2d, MAXD, TS

RPM_SCALE = 300.0


class SpeedHeightMPC:
    ny, nu = 7, 4
    name = "SH-MPC"

    def __init__(self, models, Hp=10, Hu=2,
                 Wy=(4, 3, 4, 3, .1, .1, .1), Ly=(0.2, MAXD, MAXD, 0.5, .5, .5, .5),
                 Wu=(.2, .2, .2, .02), Wdu=(.15, .15, .2, .15), Lu=(MAXD, MAXD, MAXD, RPM_SCALE),
                 tau=0.1, rate=(np.deg2rad(3),) * 3 + (250.0,), rpm_lim=(0.0, 3500.0),
                 dist_gain=1.5, z_nom=-0.3):
        self.Hp, self.Hu = Hp, Hu
        self.Ly = np.array(Ly, float)
        self.Lu = np.array(Lu, float)
        self.wy = np.array(Wy) / self.Ly
        self.wu = np.array(Wu) / self.Lu
        self.wdu = np.array(Wdu) / self.Lu
        self.rate = np.array(rate, float)
        self.rpm_lim = rpm_lim
        self.z_nom = z_nom
        self.speeds = np.array([m["U"] for m in models])
        self.grid = [self._build(m, tau, dist_gain) for m in models]
        # S[j] maps the stacked moves dU to the input at prediction step j
        self.S = np.zeros((Hp, 4, 4 * Hu))
        for j in range(Hp):
            for l in range(min(j, Hu - 1) + 1):
                self.S[j][:, 4 * l:4 * l + 4] = np.eye(4)
        self.reset()

    # ------------------------------------------------------------ model grid
    def _build(self, m, tau, gd):
        A = m["A"].copy()
        B = m["B"]
        A[5, 3] = A[5, 4] = A[5, 6] = 0.0          # same clean-up as reduce()
        Ac = np.block([[A, B[:, :3]], [np.zeros((3, 7)), -np.eye(3) / tau]])
        Bc = np.block([[np.zeros((7, 3)), B[:, 3:4]], [np.eye(3) / tau, np.zeros((3, 1))]])
        Ad, Bd = c2d(Ac, Bc, TS)
        npl = 10
        Aa = np.block([[Ad, Bd], [np.zeros((4, npl)), np.eye(4)]])
        Ba = np.vstack([Bd, np.zeros((4, 4))])
        Ca = np.hstack([np.eye(7), np.zeros((7, 7))])
        G = np.vstack([np.zeros((npl, 4)), gd * TS * np.diag(self.Lu / MAXD)])
        Qn = G @ G.T + 1e-8 * np.eye(npl + 4)
        Rn = np.diag(self.Ly ** 2)
        P = solve_discrete_are(Aa.T, Ca.T, Qn, Rn)
        M = P @ Ca.T @ np.linalg.inv(Ca @ P @ Ca.T + Rn)
        return dict(Aa=Aa, Ba=Ba, Ca=Ca, M=M, u_nom=np.r_[m["delta"], m["rpm"]])

    def _at(self, speed):
        s = float(np.clip(speed, self.speeds[0], self.speeds[-1]))
        j = int(np.clip(np.searchsorted(self.speeds, s) - 1, 0, len(self.speeds) - 2))
        a = (s - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
        g0, g1 = self.grid[j], self.grid[j + 1]
        return {k: (1 - a) * g0[k] + a * g1[k] for k in g0}, s

    def trim_at(self, speed):
        return self._at(speed)[0]["u_nom"]

    def reset(self, y0=None, u0=None):
        speed = 7.0 if y0 is None else y0[3]
        g, s = self._at(speed)
        y0 = np.array([self.z_nom, 0, 0, s, 0, 0, 0]) if y0 is None else np.asarray(y0, float)
        self.u_prev = g["u_nom"].copy() if u0 is None else np.asarray(u0, float)   # absolute
        # absolute estimate: [y (7); actuator positions (3); input disturbance (4)]
        self.xabs = np.r_[y0, self.u_prev[:3], np.zeros(4)]
        self.last = {}

    # ---------------------------------------------------------------- step
    def step(self, y, ref_seq, lock=None):
        """y: measured [z, phi, theta, u, w, p, q] (absolute).
        ref_seq: (Hp, 7) absolute output references, row i for time k+i.
        lock: length-3 array, finite entries fix that foil at the given angle [rad].
        Returns absolute command [d_FS, d_FP, d_A, rpm]."""
        y = np.asarray(y, float)
        g, s = self._at(y[3])
        Aa, Ba, Ca, M, u_nom = g["Aa"], g["Ba"], g["Ca"], g["M"], g["u_nom"]
        y_nom = np.array([self.z_nom, 0, 0, s, 0, 0, 0])
        nomvec = np.r_[y_nom, u_nom[:3], np.zeros(4)]
        Hp, Hu = self.Hp, self.Hu
        xdev = self.xabs - nomvec
        xdev = xdev + M @ ((y - y_nom) - Ca @ xdev)             # measurement update
        up = self.u_prev - u_nom
        R = np.asarray(ref_seq, float)[:Hp] - y_nom

        Apow = [np.eye(Aa.shape[0])]
        for _ in range(Hp):
            Apow.append(Aa @ Apow[-1])
        rows_H, rows_f = [], []
        Wy, Wu = np.diag(self.wy), np.diag(self.wu)
        for i in range(1, Hp + 1):
            const = Ca @ Apow[i] @ xdev
            HdU = np.zeros((7, 4 * Hu))
            for j in range(i):
                CAB = Ca @ Apow[i - 1 - j] @ Ba
                const = const + CAB @ up
                HdU += CAB @ self.S[j]
            rows_H.append(Wy @ HdU)
            rows_f.append(Wy @ (const - R[i - 1]))
        for j in range(Hp):
            rows_H.append(Wu @ self.S[j]); rows_f.append(Wu @ up)
        for l in range(Hu):
            E = np.zeros((4, 4 * Hu)); E[:, 4 * l:4 * l + 4] = np.diag(self.wdu)
            rows_H.append(E); rows_f.append(np.zeros(4))
        Hm, fm = np.vstack(rows_H), np.concatenate(rows_f)
        nU = 4 * Hu
        Pq = Hm.T @ Hm + 1e-9 * np.eye(nU)
        qv = Hm.T @ fm

        lo_abs = np.array([-MAXD, -MAXD, -MAXD, self.rpm_lim[0]])
        hi_abs = np.array([MAXD, MAXD, MAXD, self.rpm_lim[1]])
        rate_lo, rate_hi = -self.rate.copy(), self.rate.copy()
        locked = np.zeros(4, bool)
        if lock is not None:
            for i in range(3):
                if np.isfinite(lock[i]):
                    v = float(np.clip(lock[i], -MAXD, MAXD))
                    lo_abs[i] = hi_abs[i] = v
                    rate_lo[i], rate_hi[i] = -np.inf, np.inf     # operator command: no MPC rate limit
                    locked[i] = True
        Acons, lo, hi = [], [], []
        for l in range(Hu):                                      # absolute bounds on each move
            Acons.append(self.S[l]); lo.append(lo_abs - u_nom - up); hi.append(hi_abs - u_nom - up)
        Acons.append(np.eye(nU)); lo.append(np.tile(rate_lo, Hu)); hi.append(np.tile(rate_hi, Hu))
        Ac = sparse.csc_matrix(np.vstack(Acons))
        lo, hi = np.concatenate(lo), np.concatenate(hi)
        solver = osqp.OSQP()
        solver.setup(sparse.csc_matrix(np.triu(Pq)), qv, Ac, lo, hi, verbose=False,
                     eps_abs=1e-8, eps_rel=1e-8, polish=False, max_iter=20000)
        res = solver.solve()
        ok = res.x is not None and np.all(np.isfinite(res.x))
        dU = res.x if ok else np.zeros(nU)
        u_abs = np.clip(u_nom + up + dU[:4], lo_abs, hi_abs)
        # time update with the applied input, store absolute estimate
        xdev = Aa @ xdev + Ba @ (u_abs - u_nom)
        self.xabs = xdev + nomvec
        self.u_prev = u_abs
        self.last = dict(ok=ok, sched_speed=s, dist=self.xabs[10:].copy(), locked=locked)
        return u_abs
