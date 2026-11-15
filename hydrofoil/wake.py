"""Flow induced at the aft foil by the fore foils' wakes (phase 4).

Model: each fore foil is a horseshoe vortex (bound vortex + two trailing
vortices), the standard first-order lifting-line wake.  For an elliptically
loaded wing the rolled-up trailing vortices are pi/4 of the span apart.
The free surface is the high-Froude-number limit (velocity potential = 0 on
the surface, i.e. no tangential velocity there), represented by an image
horseshoe of the SAME circulation mirrored above the surface (a rigid lid
would need the opposite sign; checked numerically in scripts/phase4_tests.py).
The same-sense image is also what produces the near-surface lift loss that
the reference model's Ldec factor describes.  Fr_h = U / sqrt(g h) is about
3.3 at the design point, so this limit is reasonable.

Body axes FRD (x forward, y starboard, z down); the water moves aft.
Not validated against measurements; it is a standard estimate.
"""
import numpy as np

FAR = 2000.0      # length used for the "semi-infinite" trailing vortices [m]


def _segment(P, A, B):
    """Velocity at points P (n,3) induced by a straight vortex segment A -> B of
    unit circulation (Biot-Savart)."""
    r1 = P - A
    r2 = P - B
    n1 = np.linalg.norm(r1, axis=1)
    n2 = np.linalg.norm(r2, axis=1)
    cr = np.cross(r1, r2)
    den = n1 * n2 * (n1 * n2 + np.einsum("ij,ij->i", r1, r2))
    return cr * ((n1 + n2) / (4 * np.pi * den))[:, None]


def horseshoe(P, centre, bprime, sign=1.0):
    """Unit-circulation horseshoe vortex: bound vortex along +y through `centre`
    (which gives upward lift for water moving aft), legs trailing to x = -inf."""
    y1, y2 = centre[1] - bprime / 2, centre[1] + bprime / 2
    A = np.array([centre[0], y1, centre[2]])
    B = np.array([centre[0], y2, centre[2]])
    A_far = A - np.array([FAR, 0, 0])
    B_far = B - np.array([FAR, 0, 0])
    return sign * (_segment(P, A_far, A) + _segment(P, A, B) + _segment(P, B, B_far))


def upwash_per_gamma(fore_arm, aft_arm, span, h_fore, n_st=9):
    """Chord-weighted mean UPWASH velocity (m/s, positive = flow upwards) over
    the aft foil per unit circulation of ONE fore foil submerged h_fore."""
    bprime = np.pi / 4 * span
    # aft foil stations with elliptic chord weights
    th = (np.arange(n_st) + 0.5) / n_st * np.pi
    y = aft_arm[1] - span / 2 * np.cos(th)
    wgt = np.sin(th) ** 2
    P = np.column_stack([np.full(n_st, aft_arm[0]), y, np.full(n_st, aft_arm[2])])
    fore = np.asarray(fore_arm, float)
    v = horseshoe(P, fore, bprime, +1.0)
    image = fore.copy()
    image[2] = fore[2] - 2 * h_fore                 # mirrored above the free surface
    v += horseshoe(P, image, bprime, +1.0)          # same sense: phi = 0 on the surface
    return float(-(v[:, 2] * wgt).sum() / wgt.sum())  # FRD z is down -> upwash = -vz


class WakeTable:
    """upwash_per_gamma tabulated against fore-foil submergence."""

    def __init__(self, fore_arm, aft_arm, span):
        self.h = np.linspace(0.02, 1.2, 60)
        self.G = np.array([upwash_per_gamma(fore_arm, aft_arm, span, hh) for hh in self.h])

    def __call__(self, h):
        return float(np.interp(h, self.h, self.G))
