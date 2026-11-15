"""Operator interface, guidance and safety layer around SpeedHeightMPC.

Framework-agnostic: the same Supervisor object is driven by the Python
simulator (hydrofoil/sim.py) and by the ROS 2 node (hydrofoil_control).

Operator inputs
  speed setpoint [m/s]          default = design speed 7 m/s
  accel / decel limits [m/s^2]  the speed reference ramps at this rate
  ride height [m above water]   default 0.3
  pitch / heel trim [deg]
  mode                          AUTO | ASSIST | MANUAL
  foil overrides [deg]          per foil (FS, FP, A); NaN = not overridden

Modes
  AUTO    MPC commands everything; overrides are ignored.
  ASSIST  Overridden foils are fixed at the operator's angle; the MPC knows
          this (QP bounds) and uses the remaining foils + rpm.
  MANUAL  All three foils follow the operator (a foil without a value keeps its
          last angle); the MPC only controls rpm, and keeps estimating so that
          switching back is bumpless.
The watchdog returns control to AUTO and clears overrides if attitude or foil
submergence limits are exceeded ("FAILSAFE", latched until a mode command).
"""
from dataclasses import dataclass, field

import numpy as np

from .controllers import MAXD, TS
from .mpc4 import SpeedHeightMPC

MODES = ("AUTO", "ASSIST", "MANUAL")


@dataclass
class Limits:
    speed: tuple = (5.0, 15.0)            # range covered by the linear model array
    height: tuple = (0.15, 0.55)          # CG above still water [m]
    accel: tuple = (0.05, 3.0)
    climb_rate: float = 0.25              # ride-height reference slew [m/s]
    trim_deg: float = 5.0
    # watchdog (envelope protection) trip levels
    roll_deg: float = 10.0
    pitch_deg: float = 6.0
    min_submergence: float = 0.10         # any foil closer than this to the surface [m]
    lookahead: float = 0.6                # height is extrapolated this far ahead [s]
    height_envelope: tuple = (0.12, 0.62) # predicted CG height must stay inside [m]


class Guidance:
    """Turns operator setpoints into smooth references and previews."""

    def __init__(self, lim: Limits, speed=7.0, height=0.3):
        self.lim = lim
        self.speed_set, self.height_set = speed, height
        self.u_ref, self.h_ref = speed, height
        self.accel, self.decel = 0.5, 0.5
        self.pitch_ref = self.heel_ref = 0.0     # rad

    def _advance(self, u, h, dt):
        du = np.clip(self.speed_set - u, -self.decel * dt, self.accel * dt)
        dh = np.clip(self.height_set - h, -self.lim.climb_rate * dt, self.lim.climb_rate * dt)
        return u + du, h + dh

    def preview(self, n, dt=TS):
        """Reference rows for times k, k+1, ... k+n-1 (absolute, NED z)."""
        out = np.zeros((n, 7))
        u, h = self.u_ref, self.h_ref
        for i in range(n):
            out[i] = [-h, self.heel_ref, self.pitch_ref, u, 0, 0, 0]
            u, h = self._advance(u, h, dt)
        return out

    def step(self, dt=TS):
        self.u_ref, self.h_ref = self._advance(self.u_ref, self.h_ref, dt)

    def rebase(self, speed, height):
        """Restart the references from the craft's actual state (used when the
        MPC takes over after an upset) so that it is not asked to remove a
        large error at once; they then ramp back to the setpoints."""
        self.u_ref = float(np.clip(speed, *self.lim.speed))
        self.h_ref = float(np.clip(height, *self.lim.height))


@dataclass
class Status:
    t: float = 0.0
    mode: str = "AUTO"
    failsafe: str = ""
    warning: str = ""
    speed_set: float = 7.0
    speed_ref: float = 7.0
    speed: float = 7.0
    height_set: float = 0.3
    height_ref: float = 0.3
    height: float = 0.3
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    accel_limit: float = 0.5
    decel_limit: float = 0.5
    foil_cmd_deg: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    foil_override_deg: list = field(default_factory=lambda: [None, None, None])
    rpm_cmd: float = 0.0
    min_submergence: float = 0.0
    qp_ok: bool = True


class Supervisor:
    def __init__(self, models, limits: Limits | None = None, **mpc_kwargs):
        self.lim = limits or Limits()
        self.mpc = SpeedHeightMPC(models, **mpc_kwargs)
        self.guid = Guidance(self.lim)
        self.mode = "AUTO"
        self.override = np.full(3, np.nan)       # operator foil angles [rad]
        self.manual_hold = np.full(3, np.nan)
        self.failsafe = ""        # latched: an override was cancelled by the watchdog
        self.warning = ""         # not latched: envelope exceeded while in AUTO
        self.status = Status()
        self.cmd = self.mpc.trim_at(7.0).copy()

    # ----------------------------------------------------- operator commands
    def set_speed(self, v):
        self.guid.speed_set = float(np.clip(v, *self.lim.speed))

    def set_accel(self, accel, decel=None):
        a = float(np.clip(abs(accel), *self.lim.accel))
        d = a if decel is None else float(np.clip(abs(decel), *self.lim.accel))
        self.guid.accel, self.guid.decel = a, d

    def set_height(self, h):
        self.guid.height_set = float(np.clip(h, *self.lim.height))

    def set_trim(self, pitch_deg=None, heel_deg=None):
        L = self.lim.trim_deg
        if pitch_deg is not None:
            self.guid.pitch_ref = np.deg2rad(float(np.clip(pitch_deg, -L, L)))
        if heel_deg is not None:
            self.guid.heel_ref = np.deg2rad(float(np.clip(heel_deg, -L, L)))

    def set_mode(self, mode):
        mode = str(mode).upper()
        if mode not in MODES:
            return False
        self.failsafe = ""                        # a mode command acknowledges a failsafe
        if mode == "AUTO":
            self.override[:] = np.nan
        if mode == "MANUAL":
            # foils without an operator value hold their current command
            self.manual_hold = np.where(np.isfinite(self.override), self.override, self.cmd[:3])
        self.mode = mode
        return True

    def set_override(self, foil_deg):
        """foil_deg: 3 values [FS, FP, A] in degrees; NaN/None releases that foil."""
        v = np.array([np.nan if x is None else float(x) for x in foil_deg])
        self.override = np.where(np.isfinite(v), np.clip(np.deg2rad(v), -MAXD, MAXD), np.nan)
        if self.mode == "MANUAL":
            self.manual_hold = np.where(np.isfinite(self.override), self.override, self.manual_hold)

    # ------------------------------------------------------------- one step
    def _lock(self):
        if self.mode == "ASSIST":
            return self.override.copy()
        if self.mode == "MANUAL":
            return self.manual_hold.copy()
        return np.full(3, np.nan)

    def _h_pred(self, y):
        """CG height extrapolated `lookahead` seconds ahead with the current
        vertical speed (h_dot = u sin(theta) - w cos(theta) cos(phi))."""
        hdot = y[3] * np.sin(y[2]) - y[4] * np.cos(y[2]) * np.cos(y[1])
        return -y[0] + self.lim.lookahead * hdot

    def _watchdog(self, y, h_foils):
        L = self.lim
        why = ""
        if abs(np.rad2deg(y[1])) > L.roll_deg:
            why = f"roll {np.rad2deg(y[1]):+.1f} deg"
        elif abs(np.rad2deg(y[2])) > L.pitch_deg:
            why = f"pitch {np.rad2deg(y[2]):+.1f} deg"
        elif not (L.height_envelope[0] < self._h_pred(y) < L.height_envelope[1]):
            why = f"CG height {-y[0]:.2f} m heading to {self._h_pred(y):.2f} m"
        elif h_foils is not None and np.min(h_foils) < L.min_submergence:
            why = f"foil submergence {np.min(h_foils):.2f} m"
        if why and self.mode != "AUTO":
            self.failsafe = "override cancelled: " + why
            self.mode = "AUTO"
            self.override[:] = np.nan
            self.guid.rebase(y[3], -y[0])
        self.warning = why if (why and self.mode == "AUTO" and not self.failsafe) else ""
        return why

    def step(self, t, y, h_foils=None):
        """Call at 10 Hz.  y = [z, phi, theta, u, w, p, q]; h_foils = foil
        submergences [m] if available.  Returns [d_FS, d_FP, d_A, rpm]."""
        y = np.asarray(y, float)
        self._watchdog(y, h_foils)
        refs = self.guid.preview(self.mpc.Hp)
        self.cmd = self.mpc.step(y, refs, lock=self._lock())
        self.guid.step()
        s, g = self.status, self.guid
        s.t, s.mode, s.failsafe, s.warning = float(t), self.mode, self.failsafe, self.warning
        s.speed_set, s.speed_ref, s.speed = g.speed_set, g.u_ref, float(y[3])
        s.height_set, s.height_ref, s.height = g.height_set, g.h_ref, float(-y[0])
        s.roll_deg, s.pitch_deg = float(np.rad2deg(y[1])), float(np.rad2deg(y[2]))
        s.accel_limit, s.decel_limit = g.accel, g.decel
        s.foil_cmd_deg = [round(float(v), 3) for v in np.rad2deg(self.cmd[:3])]
        lock = self._lock()
        s.foil_override_deg = [None if not np.isfinite(v) else round(float(np.rad2deg(v)), 3) for v in lock]
        s.rpm_cmd = float(self.cmd[3])
        s.min_submergence = float(np.min(h_foils)) if h_foils is not None else float("nan")
        s.qp_ok = bool(self.mpc.last.get("ok", True))
        return self.cmd
