"""Linear (Airy) deep-water waves, thesis eqs. (2.4.8)-(2.4.12) and (2.4.14).

NED convention: z down.  Elevation zeta is positive UP; a positive zeta
increases foil submergence.  `psi` is the direction the waves travel towards
(0 = towards +x_N, i.e. following seas for a craft heading north).
"""
from dataclasses import dataclass

import numpy as np

G = 9.81


@dataclass
class WaveField:
    amp: np.ndarray      # component amplitudes zeta_a [m]
    omega: np.ndarray    # absolute frequencies [rad/s]
    psi: np.ndarray      # propagation directions [rad]
    phase: np.ndarray    # phases [rad]

    @property
    def k(self):
        return self.omega ** 2 / G          # deep-water dispersion (2.4.5)

    def at(self, xN, yN, depth, t):
        """Return (zeta_up, [u_w, v_w, w_w] NED) at a point `depth` metres
        below the mean free surface."""
        if self.amp.size == 0:
            return 0.0, np.zeros(3)
        k = self.k
        X = xN * np.cos(self.psi) + yN * np.sin(self.psi)
        ph = self.omega * t - k * X + self.phase
        decay = np.exp(-k * max(depth, 0.0))
        zeta = np.sum(self.amp * np.sin(ph))
        a = self.omega * self.amp * decay
        u = np.sum(a * np.cos(self.psi) * np.sin(ph))
        v = np.sum(a * np.sin(self.psi) * np.sin(ph))
        w = np.sum(-a * np.cos(ph))          # NED: down positive
        return zeta, np.array([u, v, w])


def calm():
    z = np.zeros(0)
    return WaveField(z, z, z, z)


def regular(amp, omega, psi=0.0, phase=0.0):
    return WaveField(np.array([amp]), np.array([omega]), np.array([psi]), np.array([phase]))


def irregular_ittc(hs, omega_peak, psi_mean=np.pi, n_freq=40, n_dir=1, spread=True, seed=0,
                   w_min=0.3, w_max=None):
    """Modified Pierson-Moskowitz (ITTC) spectrum, random phases, optional
    cos^2 spreading (2.4.16).  Mirrors what the MSS 'Waves' block does, but it
    is an independent implementation, not the MSS code."""
    rng = np.random.default_rng(seed)
    w_max = w_max or 3.5 * omega_peak
    edges = np.linspace(w_min, w_max, n_freq + 1)
    dw = edges[1] - edges[0]
    # one component per bin at a random frequency inside the bin, so the
    # realisation does not repeat with period 2*pi/dw
    w = edges[:-1] + rng.uniform(0.05, 0.95, n_freq) * dw
    T1 = 2 * np.pi / omega_peak * 0.772         # peak period -> mean period
    A = 173 * hs ** 2 / T1 ** 4
    B = 691 / T1 ** 4
    S = A * w ** -5 * np.exp(-B * w ** -4)
    if n_dir == 1 or not spread:
        psis = np.array([psi_mean]); D = np.array([1.0]); dpsi = 1.0
    else:
        rel = np.linspace(-np.pi / 2, np.pi / 2, n_dir + 2)[1:-1]
        dpsi = rel[1] - rel[0]
        D = 2 / np.pi * np.cos(rel) ** 2
        psis = psi_mean + rel
    W, P = np.meshgrid(w, psis, indexing="ij")
    amp = np.sqrt(2 * S[:, None] * D[None, :] * dw * dpsi)
    ph = rng.uniform(0, 2 * np.pi, amp.shape)
    return WaveField(amp.ravel(), W.ravel(), P.ravel(), ph.ravel())


def _smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class SeaState:
    """A sea that can be changed while running.  Each new wave field fades in
    over `ramp` seconds while the previous one fades out, so elevation and
    orbital velocities stay continuous.  Same interface as WaveField.at()."""

    def __init__(self):
        self.sets = []          # [field, t_in, ramp_in, t_out, ramp_out]

    def set(self, field, t, ramp=5.0):
        ramp = max(ramp, 1e-3)
        for e in self.sets:
            if e[3] is None:
                e[3], e[4] = t, ramp
        self.sets = [e for e in self.sets if not (e[3] is not None and t > e[3] + e[4])]
        if field is not None and field.amp.size:
            self.sets.append([field, t, ramp, None, ramp])

    def _weight(self, e, t):
        w = _smooth((t - e[1]) / e[2])
        if e[3] is not None:
            w *= 1.0 - _smooth((t - e[3]) / e[4])
        return w

    def at(self, xN, yN, depth, t):
        zeta, vel = 0.0, np.zeros(3)
        for e in self.sets:
            w = self._weight(e, t)
            if w > 0.0:
                z, v = e[0].at(xN, yN, depth, t)
                zeta += w * z
                vel += w * v
        return zeta, vel
