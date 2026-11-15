"""Trim + Jacobian linearisation, the Python counterpart of
find_operpoint.m / find_linear_model.m / make_linmodel_array.m.

Trim spec copied from find_operpoint.m: z = -0.3 m, phi = theta = 0, u = U0
known; solve the three foil deflections and propeller rpm so that
u_dot = w_dot = p_dot = q_dot = 0 (and z_dot = 0, hence w = 0).
"""
import numpy as np
from scipy.optimize import fsolve

from .plant import Hydrofoil

# linear-model state order used in the thesis / repo: [z, phi, theta, u, w, p, q]
LIN_IDX = [1, 2, 3, 4, 5, 6, 7]
LIN_NAMES = ["z", "phi", "theta", "u", "w", "p", "q"]


def trim(model: Hydrofoil, U0=7.0, z0=-0.3, guess=(0.05, 0.05, 1500.0)):
    core = np.array([0.0, z0, 0.0, 0.0, U0, 0.0, 0.0, 0.0])

    def res(v):
        dF, dA, rpm = v
        d = model.deriv_simplified(core, np.array([dF, dF, dA]), rpm)
        return [d[4], d[5], d[7]]          # u_dot, w_dot, q_dot  (p_dot = 0 by symmetry)

    sol, info, ier, msg = fsolve(res, guess, full_output=True, xtol=1e-12)
    if ier != 1:
        raise RuntimeError(f"trim failed at U0={U0}: {msg}")
    dF, dA, rpm = sol
    return core, np.array([dF, dF, dA]), rpm


def linearize(model: Hydrofoil, core0, delta0, rpm0, eps=1e-6):
    """Central-difference Jacobians of the simplified model.
    Returns A (7x7) over [z,phi,theta,u,w,p,q] and B (7x4) over
    [d_FS, d_FP, d_A, rpm]."""
    n = len(LIN_IDX)
    A = np.zeros((n, n))
    B = np.zeros((n, 4))
    u0 = np.r_[delta0, rpm0]

    def f(core, uu):
        return model.deriv_simplified(core, uu[:3], uu[3])[LIN_IDX]

    for j, k in enumerate(LIN_IDX):
        h = eps * max(1.0, abs(core0[k]))
        cp, cm = core0.copy(), core0.copy()
        cp[k] += h; cm[k] -= h
        A[:, j] = (f(cp, u0) - f(cm, u0)) / (2 * h)
    for j in range(4):
        h = eps * max(1.0, abs(u0[j])) * (100 if j == 3 else 1)
        up, um = u0.copy(), u0.copy()
        up[j] += h; um[j] -= h
        B[:, j] = (f(core0, up) - f(core0, um)) / (2 * h)
    return A, B


def reduce(A, B):
    """Drop the surge state and the rpm input -> 6-state model
    [z, phi, theta, w, p, q] with 3 foil inputs (make_linmodel_array.m),
    including its clean-up of three numerically tiny p-row entries."""
    A = A.copy()
    A[5, 3] = A[5, 4] = A[5, 6] = 0.0     # MATLAB: A(6,4)=A(6,5)=A(6,7)=0
    keep = [0, 1, 2, 4, 5, 6]
    return A[np.ix_(keep, keep)], B[np.ix_(keep, [0, 1, 2])]


def model_array(model, speeds, z0=-0.3):
    out = []
    guess = (0.05, 0.05, 1500.0)
    for U in speeds:
        core, delta, rpm = trim(model, U, z0, guess)
        guess = (delta[0], delta[2], rpm)
        A, B = linearize(model, core, delta, rpm)
        Ar, Br = reduce(A, B)
        out.append(dict(U=U, core=core, delta=delta, rpm=rpm, A=A, B=B, Ar=Ar, Br=Br))
    return out
