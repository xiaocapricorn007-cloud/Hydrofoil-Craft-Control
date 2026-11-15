"""Nominal vessel parameters.

Every number here is copied from the original MATLAB/Simulink sources of
jskoun/MPC_HydrofoilControl (src/implementation_dynamicmodel/model_parameters.m
and the block parameters inside hydrofoil_sim_unified.slx), or from Table A.1 of
Skounakis-Kounavis, "Dynamic Modeling & Predictive Control of a Hydrofoil
Vessel", NTUA diploma thesis, 2025.  Where a value is NOT in those sources it is
flagged ASSUMPTION.
"""
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.integrate import quad

DATA = Path(__file__).resolve().parent.parent / "data"


def _polar_63412():
    # XFLR5 3D LLT polar of the elliptic NACA 63-412 wing at 7 m/s (repo csv).
    # columns: alpha, Beta, CL, CDi, CDv, CD, CY, Cl, Cm, ...
    d = np.genfromtxt(DATA / "63412_elwing_7ms.csv", delimiter=",", skip_header=1)
    return dict(alpha=d[:, 0], cl=d[:, 2], cdi=d[:, 3], cdv=d[:, 4], cm=d[:, 8])


def _polar_0012():
    # XFOIL NACA 0012 at Re=500k (strut section); col 0 alpha, col 2 cd
    d = np.genfromtxt(DATA / "0012_foil_Re500.csv", delimiter=",")
    return dict(alpha=d[:, 0], cd=d[:, 2])


@dataclass
class Physics:
    """Phase 4 physics upgrades.  All False = the reference model (the validated
    port of the Simulink model).  Each switch is independent."""
    added_mass: bool = False      # foil + strut added mass in heave / roll / pitch
    speed_scaling: bool = False   # unsteady-lift poles and wave-drag Froude number use the actual speed
    lift_lag_only: bool = False   # lag only circulatory lift (Wagner for motion, Kussner for waves)
    roll_inflow: bool = False     # roll rate enters the foil inflow; strip-theory roll damping
    ventilation: bool = False     # lift collapse near the surface, with hysteresis
    downwash: bool = False        # fore-foil wake induces flow at the aft foil
    actuator_rate: bool = False   # servo rate limit

    @classmethod
    def v2(cls):
        return cls(*([True] * 7))

    def any(self):
        return any(vars(self).values())

    # bit i of the mask = i-th field above (same as libfoil hf::Physics::mask)
    def mask(self):
        return sum(1 << i for i, v in enumerate(vars(self).values()) if v)

    @classmethod
    def from_mask(cls, m):
        return cls(*[bool(int(m) >> i & 1) for i in range(7)])

    @classmethod
    def parse(cls, v):
        """'ref' | 'v2' | mask number | 'name,name' | {name: bool} | Physics."""
        if isinstance(v, cls):
            return v
        if isinstance(v, dict):
            bad = set(v) - set(vars(cls()))
            if bad:
                raise ValueError(f"unknown physics switch {sorted(bad)}")
            return cls(**{k: bool(x) for k, x in v.items()})
        if isinstance(v, (int, float)):
            return cls.from_mask(int(v))
        t = str(v).replace(" ", "").lower()
        if t in ("", "ref", "reference", "off"):
            return cls()
        if t in ("v2", "all", "on"):
            return cls.v2()
        if t.isdigit():
            return cls.from_mask(int(t))
        names = t.split(",")
        bad = set(names) - set(vars(cls()))
        if bad:
            raise ValueError(f"unknown physics switch {sorted(bad)}")
        return cls(**{k: True for k in names})

    def __str__(self):
        m = self.mask()
        if m == 0:
            return "ref"
        if m == 127:
            return "v2"
        return ",".join(k for k, v in vars(self).items() if v)


@dataclass
class Params:
    phys: Physics = field(default_factory=Physics)

    # --- phase 4 parameters that are NOT in the reference -----------------
    # Ventilation.  ASSUMPTION: no data for this foil.  Lift drops to
    # vent_lift_factor of its wetted value when the foil is shallower than
    # vent_h_on chords, and only recovers once deeper than vent_h_off chords.
    vent_h_on: float = 0.5
    vent_h_off: float = 1.5
    vent_lift_factor: float = 0.3
    vent_tau_on: float = 0.03
    vent_tau_off: float = 0.25
    # Servo rate limit [rad/s].  ASSUMPTION (the reference has a pure lag).
    act_rate: float = np.deg2rad(60.0)
    # Strut section lift slope used for strut roll damping (thin-foil theory).
    strut_cla: float = 2 * np.pi

    # --- rigid body (model_parameters.m) ---
    mass: float = 215.0
    g: float = 9.81
    rho: float = 1024.8
    Ixx: float = 31.0
    Iyy: float = 503.0
    Izz: float = 1.0

    # --- foil arms from CG, body axes x fwd, y stbd, z down (Table A.1) ---
    arm_FS: tuple = (1.1, 0.62, 0.75)    # fore starboard
    arm_FP: tuple = (1.1, -0.62, 0.75)   # fore port
    arm_A: tuple = (-2.45, 0.0, 0.96)    # aft (elevator)

    S_F: float = 0.048        # fore foil planform area [m^2]
    S_A: float = 0.046        # aft foil planform area
    span: float = 0.76        # all foils
    mac: float = 0.08         # chord / MAC used for free-surface ratio
    t_foil: float = 0.12 * 0.08   # foil thickness (12 % of chord)
    cs_F: float = 0.08        # strut chord fore
    cs_A: float = 0.10        # strut chord aft

    # --- propeller (Table A.1 / model_parameters.m) ---
    prop_D: float = 0.3
    prop_wake: float = 0.05
    prop_arm_z: float = 0.96   # zpr = dza

    # --- Wagner (R.T. Jones) unsteady lift, model_parameters.m ---
    usted: float = 7.0         # nominal speed used inside some blocks
    wagC1: float = 0.165
    wagC2: float = 0.335

    # --- actuator: first-order lag, thesis eq. (2.5.4), tau = 0.1 s.
    # The repo's Simulink gain 'actuation_tau' is never defined in any .m file,
    # so the thesis value is used here.
    act_tau: float = 0.1
    max_deflect: float = np.deg2rad(10.0)

    # --- speed loop: discrete PI read from hydrofoil_sim_unified.slx ---
    rpm_trim_ff: float = 1200.0   # 'trimRPM' constant
    pi_P: float = 250.0
    pi_I: float = 200.0
    pi_Ts: float = 0.01

    # --- air drag on hull + solar deck (phase 3).  ASSUMPTION: the reference
    # model neglects air forces and gives no above-water geometry.  Values are
    # Cd*A estimates for a 5.6 x 1.5 m open hull with a seated pilot:
    # frontal ~0.6 m^2 x Cd 0.8, side ~1.4 m^2 x Cd 1.0, centre of effort 0.35 m
    # above the CG.
    air_CdA_x: float = 0.5
    air_CdA_y: float = 1.4
    air_z_ce: float = 0.35

    polar: dict = field(default_factory=_polar_63412)
    strut_polar: dict = field(default_factory=_polar_0012)

    def __post_init__(self):
        kt = np.genfromtxt(DATA / "prop_kt_data.csv", delimiter=",")
        kq = np.genfromtxt(DATA / "prop_kq_data.csv", delimiter=",")
        self.KT_poly = np.polyfit(kt[:, 0], kt[:, 1], 2)
        self.KQ_poly = np.polyfit(kq[:, 0], kq[:, 1], 2)
        # strut drag coefficient at zero incidence (constant 'a' = 0 in the slx)
        self.cd_strut0 = float(np.interp(0.0, self.strut_polar["alpha"], self.strut_polar["cd"]))
        # roll-damping span integrals, exactly as in model_parameters.m
        dyf = self.arm_FS[1]
        f = lambda y, yc: y ** 2 * 0.08 * np.sqrt(max(0.0, 1 - ((y - yc) / 0.4) ** 2))
        self.L_FS = quad(f, dyf - 0.4, dyf + 0.4, args=(dyf,))[0]
        self.L_FP = quad(f, -dyf - 0.4, -dyf + 0.4, args=(-dyf,))[0]
        self.L_A = 0.08 * np.pi * 0.8 ** 3 / 64
        # Wagner pole locations scaled to model dimensions (model_parameters.m)
        self.wag_e1 = 0.0455 * (self.usted / (self.mac / 2))
        self.wag_e2 = 0.3 * (self.usted / (self.mac / 2))
        self._added_mass()

    def _added_mass(self):
        """Added mass of foils and struts (thesis eqs. 2.5.33-2.5.37, which the
        Simulink model derives but never applies).

        Foil: strip of elliptic sections, a33 = pi rho c^2 / 4 per unit span, over an
        elliptic planform -> A33 = pi rho b c0^2 / 6, times Scherer's aspect-ratio
        factor AR / sqrt(1 + AR^2).  Strut: a22 = pi rho cs^2 / 4 per unit depth for
        sideways motion, which with sway locked only adds roll inertia; evaluated
        at the design ride height (CG 0.3 m above the water).
        Result: MA, a 3x3 matrix over [w, p, q] about the CG (body FRD)."""
        ar = lambda S: self.span ** 2 / S
        MA = np.zeros((3, 3))
        self.A33 = []
        for arm, S, cs in ((self.arm_FS, self.S_F, self.cs_F), (self.arm_FP, self.S_F, self.cs_F),
                           (self.arm_A, self.S_A, self.cs_A)):
            lx, ly, lz = arm
            c0 = 4 * S / (np.pi * self.span)                     # root chord of the ellipse
            a33 = np.pi * self.rho * self.span * c0 ** 2 / 6 * ar(S) / np.sqrt(1 + ar(S) ** 2)
            self.A33.append(a33)
            j = np.array([1.0, ly, -lx])                         # foil z-velocity = w + p ly - q lx
            MA += a33 * np.outer(j, j)
            MA[1, 1] += np.pi * self.rho * c0 ** 2 * self.span ** 3 / 120      # own-span roll term
            z_wl = 0.3                                           # waterline below the CG at design height
            MA[1, 1] += np.pi * self.rho * cs ** 2 / 4 * (lz ** 3 - z_wl ** 3) / 3
        self.MA = MA

    def foils(self):
        """name, arm, area, strut chord, roll-damping integral"""
        return [
            ("FS", np.array(self.arm_FS), self.S_F, self.cs_F, self.L_FS),
            ("FP", np.array(self.arm_FP), self.S_F, self.cs_F, self.L_FP),
            ("A", np.array(self.arm_A), self.S_A, self.cs_A, self.L_A),
        ]
