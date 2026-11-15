"""Operator-settable environment (phase 3): sea state, wind, current, water.

One Environment object is the single source of truth.  It is used directly by
the Python simulator, and by the ROS env node, which forwards the same wave
components / wind / current / density to the Gazebo plugin.

Operator conventions (craft heading = north, fixed by the sway/yaw lock):
  sea / wind direction : where it comes FROM, degrees clockwise from the bow
                         (0 = head sea / headwind, 90 = from starboard, 180 = following)
  current direction    : where it flows TO (0 = with the craft, 180 = against it)
"""
import numpy as np

from .params import Physics

from .plant import Env
from .waves import WaveField, SeaState, irregular_ittc, regular

LIMITS = dict(hs=(0.0, 1.0), tp=(1.0, 12.0), amp=(0.0, 0.5), omega=(0.3, 9.0),
              wind=(0.0, 25.0), gust=(0.0, 0.6), current=(0.0, 3.0), rho=(990.0, 1040.0))


def seawater_density(temp_c, salinity_psu):
    """One-atmosphere international equation of state of seawater
    (UNESCO 1983 / Millero and Poisson 1981).  15 C, 35 psu -> 1025.97 kg/m^3."""
    T, S = float(temp_c), float(salinity_psu)
    rw = (999.842594 + 6.793952e-2 * T - 9.095290e-3 * T ** 2 + 1.001685e-4 * T ** 3
          - 1.120083e-6 * T ** 4 + 6.536332e-9 * T ** 5)
    A = 8.24493e-1 - 4.0899e-3 * T + 7.6438e-5 * T ** 2 - 8.2467e-7 * T ** 3 + 5.3875e-9 * T ** 4
    B = -5.72466e-3 + 1.0227e-4 * T - 1.6546e-6 * T ** 2
    return rw + A * S + B * S ** 1.5 + 4.8314e-4 * S ** 2


def _clip(v, key):
    return float(np.clip(float(v), *LIMITS[key]))


class Environment:
    def __init__(self, seed=0, aero=True, rho=1024.8):
        self.seed = int(seed)
        self.aero = bool(aero)
        self.sea = dict(kind="calm", hs=0.0, tp=0.0, dir=0.0, spread=False, amp=0.0, omega=0.0, ramp=5.0)
        self.wind = dict(speed=0.0, dir=0.0, gust=0.0)
        self.current = dict(speed=0.0, dir=0.0)
        self.water = dict(rho=float(rho), temp=None, salinity=None)
        self.sea_state = SeaState()          # blended wave field used by the Python plant
        self.wave_id = 0                     # increments every time the sea is changed
        self.field = None                    # latest target WaveField
        self._gust, self._k = 0.0, -1
        self.wind_now = np.zeros(2)
        self.physics = None                  # plant Physics switches; None = leave the plant as built

    # ------------------------------------------------------------ operator
    def apply(self, upd, t=0.0):
        """upd: dict with any of the keys sea, wind, current, water, seed.
        Returns True if the wave field changed."""
        changed = False
        if "physics" in upd:
            self.physics = Physics.parse(upd["physics"])
        if "seed" in upd:
            self.seed = int(upd["seed"])
        if "wind" in upd:
            w = upd["wind"]
            self.wind.update(speed=_clip(w.get("speed", self.wind["speed"]), "wind"),
                             dir=float(w.get("dir", self.wind["dir"])) % 360.0,
                             gust=_clip(w.get("gust", self.wind["gust"]), "gust"))
        if "current" in upd:
            c = upd["current"]
            self.current.update(speed=_clip(c.get("speed", self.current["speed"]), "current"),
                                dir=float(c.get("dir", self.current["dir"])) % 360.0)
        if "water" in upd:
            w = upd["water"]
            if "rho" in w:
                self.water.update(rho=_clip(w["rho"], "rho"), temp=None, salinity=None)
            elif "temp" in w:
                T, S = float(w["temp"]), float(w.get("salinity", 35.0))
                self.water.update(rho=_clip(seawater_density(T, S), "rho"), temp=T, salinity=S)
        if "sea" in upd:
            u = upd["sea"]
            s = dict(self.sea)
            s.update(u)
            kind = str(u.get("kind", "")).lower()
            if not kind:
                kind = "regular" if "amp" in u else ("irregular" if "hs" in u else s["kind"])
            s["hs"], s["amp"] = _clip(s["hs"], "hs"), _clip(s["amp"], "amp")
            s["dir"] = float(s["dir"]) % 360.0
            if kind == "irregular":
                s["tp"] = _clip(s["tp"] if s["tp"] else 2.5, "tp")
                if s["hs"] == 0:
                    kind = "calm"
            elif kind == "regular":
                s["omega"] = _clip(s["omega"] if s["omega"] else 3.0, "omega")
                if s["amp"] == 0:
                    kind = "calm"
            else:
                kind = "calm"
            s["kind"] = kind
            self.sea = s
            self.field = self._make_field()
            self.sea_state.set(self.field, t, float(s.get("ramp", 5.0)))
            self.wave_id += 1
            changed = True
        return changed

    def _make_field(self):
        s = self.sea
        psi = np.deg2rad(s["dir"] + 180.0)          # FROM -> direction of travel
        if s["kind"] == "regular":
            return regular(s["amp"], s["omega"], psi, 0.0)
        if s["kind"] == "irregular":
            return irregular_ittc(s["hs"], 2 * np.pi / s["tp"], psi_mean=psi, n_freq=36,
                                  n_dir=5 if s.get("spread") else 1, spread=bool(s.get("spread")),
                                  seed=self.seed + self.wave_id)
        z = np.zeros(0)
        return WaveField(z, z, z, z)

    def wave_components(self):
        """(N, 4) array [amp, omega, psi, phase] of the latest target sea."""
        f = self.field
        if f is None or f.amp.size == 0:
            return np.zeros((0, 4))
        return np.column_stack([f.amp, f.omega, f.psi, f.phase])

    # ------------------------------------------------------------- stepping
    def step(self, t, dt=0.1):
        """Advance the gust process to time t (call at >= 10 Hz) and return
        the wind velocity over ground, NED [north, east]."""
        k = int(np.floor(t / dt + 1e-9))
        tau = 3.0
        a = np.exp(-dt / tau)
        while self._k < k:
            self._k += 1
            n = np.random.default_rng([self.seed, 7919, self._k]).standard_normal()
            self._gust = a * self._gust + np.sqrt(1 - a * a) * n
        v = max(0.0, self.wind["speed"] * (1.0 + self.wind["gust"] * self._gust))
        d = np.deg2rad(self.wind["dir"])
        self.wind_now = -v * np.array([np.cos(d), np.sin(d)])     # blows FROM dir
        return self.wind_now

    def current_vec(self):
        d = np.deg2rad(self.current["dir"])
        return self.current["speed"] * np.array([np.cos(d), np.sin(d)])

    def plant_env(self):
        return Env(rho=self.water["rho"], current=tuple(self.current_vec()),
                   wind=tuple(self.wind_now), aero=self.aero)

    def status(self):
        f = self.field
        hs_real = 4 * np.sqrt(np.sum(f.amp ** 2) / 2) if f is not None and f.amp.size else 0.0
        return dict(sea=dict(self.sea, hs_realised=round(float(hs_real), 4),
                             components=int(f.amp.size) if f is not None else 0),
                    wind=dict(self.wind, now=[round(float(x), 3) for x in self.wind_now]),
                    current=dict(self.current), water=dict(self.water), aero=self.aero,
                    wave_id=self.wave_id, seed=self.seed,
                    physics=None if self.physics is None else str(self.physics))
