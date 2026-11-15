"""Builds the linear model array and the controller set used in the thesis
(final designs: LQI-SW, MPC-IP, MPC-IDP, LPV-MPC-IDP, plus the non-preview
MPC-I / MPC-ID used in Table 5.4)."""
import numpy as np

from .plant import Hydrofoil
from .linearize import trim, linearize, reduce
from .controllers import LQISW, MPC

SPEEDS = np.linspace(5, 15, 21)     # make_linmodel_array.m
NOMINAL_IDX = 4                      # nominal_idx = 5 (MATLAB) -> 7 m/s


def build_models(thesis_consistent=True, params=None):
    """thesis_consistent: zero the d/dz column of A.  The repo's Simulink
    linearisation (Appendix A.3) has an all-zero z column, i.e. its controllers
    were designed without the free-surface height sensitivity that the
    nonlinear plant contains.  We reproduce that design choice.
    params: linearise a different plant (e.g. Params(phys=Physics.v2()));
    default = the reference model."""
    lin = Hydrofoil(params=params, wagner=False)
    out, guess = [], (0.05, 0.05, 1500.0)
    for U in SPEEDS:
        core, delta, rpm = trim(lin, U, -0.3, guess)
        guess = (delta[0], delta[2], rpm)
        A, B = linearize(lin, core, delta, rpm)
        if thesis_consistent:
            A[:, 0] = 0.0
        Ar, Br = reduce(A, B)
        out.append(dict(U=U, core=core, delta=delta, rpm=rpm, A=A, B=B, Ar=Ar, Br=Br))
    return out


def build_controllers(models, which=("LQI-SW", "MPC-I", "MPC-ID", "MPC-IP", "MPC-IDP", "LPV-MPC-IDP")):
    nom = models[NOMINAL_IDX]
    table = {
        "LQI-SW": lambda: LQISW(nom["Ar"], nom["Br"], nom["delta"]),
        "LQI-S": lambda: LQISW(nom["Ar"], nom["Br"], nom["delta"], antiwindup=False),
        "LQI": lambda: LQISW(nom["Ar"], nom["Br"], nom["delta"], saturate=False, antiwindup=False),
        "MPC-I": lambda: MPC(models, NOMINAL_IDX),
        "MPC-ID": lambda: MPC(models, NOMINAL_IDX, delay=True),
        "MPC-IP": lambda: MPC(models, NOMINAL_IDX, preview=True),
        "MPC-IDP": lambda: MPC(models, NOMINAL_IDX, delay=True, preview=True),
        "LPV-MPC-IP": lambda: MPC(models, NOMINAL_IDX, preview=True, lpv=True),
        "LPV-MPC-IDP": lambda: MPC(models, NOMINAL_IDX, delay=True, preview=True, lpv=True),
    }
    return {k: table[k]() for k in which}
