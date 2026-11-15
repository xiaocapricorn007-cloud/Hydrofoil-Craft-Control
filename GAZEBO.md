# Gazebo simulator: how to run it

Stack: WSL2 Ubuntu 26.04, ROS 2 Lyrical, Gazebo Jetty (gz-sim 10.5).
Sources live on the Windows side in `ros_ws/src`; the build goes to
`~/hydrofoil_build` inside WSL.

## One-time setup
`$REPO` below is where this repository is seen from WSL, e.g.
`/mnt/c/Users/<you>/Hydrofoil-Craft-Control` (`export REPO=...`).

```bash
# as root: installs ROS 2 Lyrical, the Gazebo bridge, build tools, pip, osqp
wsl -d Ubuntu -u root -- bash $REPO/setup/phase0_install.sh
# as the normal user: build the workspace and check the install
wsl -d Ubuntu -- bash $REPO/setup/build_ws.sh
wsl -d Ubuntu -- bash $REPO/setup/phase0_check.sh
```

## Run (inside a WSL terminal)
```bash
source $REPO/setup/env.sh    # every new terminal
ros2 launch hydrofoil_bringup sim.launch.py             # Gazebo window + controller
#   add  headless:=true  to run without the window
#   add  physics:=v2     for the phase 4 plant (default ref = validated reference model)
#   add  follow:=false   for a free camera
```
The GUI camera follows the craft automatically (FOLLOW mode, 7.5 m behind,
4 m to port, 2.4 m up; checked with `setup/check_follow.sh` and `setup/diag_run.sh`).  The launch sends
the follow request 4 times over the first ~12 s while the GUI starts up.
For a free camera, launch with `follow:=false`.
The craft starts foiling at the design speed (7 m/s) and 0.3 m ride height
with the MPC in AUTO mode.

In a second terminal (after `source .../env.sh`), drive it with `hf`:
```bash
alias hf="ros2 run hydrofoil_control hf"
hf watch                 # live status line
hf speed 10              # speed setpoint, 5..15 m/s
hf accel 0.8 0.5         # acceleration / deceleration limits, m/s^2
hf height 0.40           # ride height (CG above water), 0.15..0.55 m
hf pitch 1.0             # pitch trim, deg        hf heel -2   heel trim, deg
hf mode assist           # AUTO | ASSIST | MANUAL
hf foil a 3.5            # fix the aft foil at 3.5 deg (fs | fp | a)
hf foil all 3.3 3.3 3.0  # fix all three
hf foil off              # release all foils
hf mode auto             # back to full automatic (also clears a failsafe)
```
Environment (takes effect while running; a new sea fades in over 5 s):
```bash
hf env                       # show the current environment
hf sea 0.4 3.0 0             # irregular sea: Hs 0.4 m, peak period 3 s, head sea
hf sea 0.4 3.0 45 spread     # from 45 deg off the starboard bow, short-crested
hf sea regular 0.1 3.0 0     # regular wave: amplitude 0.1 m, 3 rad/s
hf sea calm
hf wind 10 90 0.2            # 10 m/s from the starboard beam, 20 % gusts
hf current 0.5 180           # 0.5 m/s flowing against the craft
hf water 25 0                # 25 C fresh water  -> density 997.0 kg/m^3
hf rho 1024.8                # or set the density directly
```
Directions: sea and wind are where they come FROM, in degrees clockwise from
the bow (0 = head-on, 90 = starboard beam, 180 = from astern).  Current is
where it flows TO (0 = with the craft, 180 = against).
Limits: Hs up to 1 m, wind up to 25 m/s, current up to 3 m/s, density 990..1040.

Plant fidelity (phase 4, can be changed in flight):
```bash
hf physics v2                     # all phase 4 upgrades
hf physics ref                    # back to the validated reference model
hf physics downwash,ventilation   # any combination of the switches below
```

Stop everything: `bash $REPO/setup/stop_sim.sh`

## Modes
| Mode | Foils | Propeller rpm |
|---|---|---|
| AUTO | MPC | MPC |
| ASSIST | Overridden foils are fixed at your angle; the MPC knows and uses the others | MPC |
| MANUAL | All three follow you (a foil without a value holds its last angle) | MPC |

There are 3 foils and 3 things to hold (height, roll, pitch), so every foil
you fix removes one degree of freedom: the MPC keeps height first and lets
pitch or roll drift.  Example: aft foil +1 deg above trim gives about -1 deg
pitch at the same height.

Envelope protection (watchdog) cancels overrides and returns to AUTO when
roll > 10 deg, pitch > 6 deg, a foil is within 0.10 m of the surface, or the
CG height extrapolated 0.6 s ahead leaves 0.12..0.62 m.  It shows up in
`hf status` as `!! override cancelled: ...` until the next `hf mode` command.

## What is where
| Path | What |
|---|---|
| `ros_ws/src/hydrofoil_physics` | `libfoil` (C++ force model), `FoilDynamics` Gazebo plugin, craft model, golden test |
| `ros_ws/src/hydrofoil_control` | `control_node` (operator-driven), `parity_node` (phase 1 check), `hf` CLI |
| `ros_ws/src/hydrofoil_bringup` | worlds, bridge config, launch files |
| `hydrofoil/mpc4.py` | speed + height MPC (7 outputs, 3 foils + rpm, scheduled on speed) |
| `hydrofoil/supervisor.py` | guidance (accel-limited speed, ride height), modes, overrides, watchdog |
| `hydrofoil/environment.py` | sea state, wind, current, water density (shared by both simulators) |
| `ros_ws/src/hydrofoil_control/.../env_node.py` | forwards the environment to the Gazebo plugin |
| `scenarios/*.json` | scripted operator + environment sequences for repeatable runs |
| `logs/` | trajectories saved by the scripted Gazebo runs |
| `setup/run_parity.sh`, `setup/run_phase2.sh` | headless acceptance runs |
| `scripts/compare_gz.py`, `scripts/compare_phase2.py` | Gazebo vs Python-simulator comparison |

## How the physics is split
Gazebo integrates the rigid body (1 ms step) and applies gravity.  The
`FoilDynamics` plugin applies everything hydrodynamic: per-foil lift/drag from
the polars, free-surface corrections, Wagner unsteady lift, struts, roll
damping, propeller thrust, actuator lag.  This is the same model as
`hydrofoil/plant.py`; `libfoil` matches it to 1e-13 on 60 test states.
Sway, yaw rate and heading are held at zero by a stiff virtual constraint
(the reference model deletes those degrees of freedom).

## Environment model (phase 3)
| Item | Model | Source / status |
|---|---|---|
| Waves | Linear deep-water waves, sum of components. Irregular = ITTC / modified Pierson-Moskowitz spectrum from Hs and Tp, 36 frequencies (x5 directions with `spread`, cos^2 spreading), random phases from the seed | Standard theory; checked: realised Hs within 2 % of the setting |
| Sea changes | Old field fades out and new one fades in over `ramp` seconds | Ours; no jump in elevation |
| Current | Uniform water velocity; foils, struts and propeller see speed through the water | Ours |
| Water density | Set directly, or from temperature + salinity (UNESCO 1983 equation of state) | Checked against the reference value 1025.97 at 15 C / 35 psu |
| Wind | Quadratic air drag on hull and deck, with a heeling/pitching moment; gusts are a first-order random process | **Assumed** Cd x area (0.5 m^2 frontal, 1.4 m^2 side, centre 0.35 m above CG): the reference neglects air forces |
| Sonar | Range from a hull-fixed downward sensor to the instantaneous wave surface, 20 Hz, 2 mm noise (`/hydrofoil/debug[19]`) | Geometry from the thesis; not yet used by the controller |
| Water surface you see | Drawn by the physics plugin from the same wave field the foils feel (a 48 x 26 m patch that follows the craft) | Same object, so they cannot disagree |

One `Environment` object (`hydrofoil/environment.py`) generates the wave
components, gusts and density.  The Python simulator uses it directly; the
`env_node` forwards the same numbers to the Gazebo plugin.  Wave time and
wave x = 0 are both anchored at the moment/place the craft is released, so a
scripted scenario gives the same sea in both simulators.

Known limits of this phase: water temperature only changes density (the foil
polars are for one Reynolds number); with sway and yaw locked, beam seas and
beam winds only act through roll; no hull, so nothing happens if the hull
would touch a crest.

## Physics upgrades (phase 4)
`ref` is the validated port of the reference Simulink model.  `v2` switches on
all of the switches below.  Each switch also works on its own.  The same code
exists in Python (`hydrofoil/plant.py`, `Params(phys=Physics(...))`) and C++
(libfoil, `hf::Physics`), and they agree to about 1e-14 (golden test, 144 cases).

| Switch | What it does | Effect at 7 m/s | Status |
|---|---|---|---|
| `added_mass` | Foil added mass (strip theory, elliptic sections, Scherer aspect-ratio factor) and strut added mass, coupled over heave/roll/pitch; reaction to foil rotation, -A33 U dδ/dt | +3.6 % heave, +19 % roll, +4.1 % pitch inertia | Thesis eqs. 2.5.33-37 (derived in the thesis, never applied in the Simulink model). In Gazebo it is an exact algebraic transform of the applied wrench, with no one-step lag |
| `speed_scaling` | Wagner poles and wave-drag Froude number follow the actual speed | Wagner time constants 176/27 ms at 5 m/s, 73/11 ms at 12 m/s (the reference fixes them at the 7 m/s values) | Standard theory |
| `lift_lag_only` | Only circulatory lift is lagged.  Motion uses Wagner; wave gusts use Kussner (b = 0.13, 1.0) | Not measured on its own; the v2 runs in waves differ from ref by a few % | Standard theory |
| `roll_inflow` | Roll rate enters each foil's inflow (p y); own-span roll damping from strip theory with the local polar slope | Roll damping -6 % | Standard theory |
| `ventilation` | Lift drops to 30 % when shallower than 0.5 chord and recovers only deeper than 1.5 chords | Hs 0.5 at 0.55 m ride height: heave std 8.5 -> 13.4 cm | **Assumed numbers**, no data for this foil |
| `downwash` | Fore-foil wakes as horseshoe vortices with a free-surface image (high-Froude limit, phi = 0 on the surface), lagged by the travel time to the aft foil | Aft foil +0.21 deg incidence, aft trim 2.985 -> 2.771 deg | Standard estimate, **not validated** against measurements |
| `actuator_rate` | Servo rate limit | 60 deg/s | **Assumed** (the reference has a pure lag) |

On the hold rail, the plugin solves the trim of the active physics itself.
The trim matches Python to 1e-12.  Test results are in `logs/phase4_tests.txt`
(`python scripts/phase4_tests.py`).  Gazebo vs Python with `v2`: calm tour
z max diff 1.5 cm (rms 0.15 cm); env_tour with waves z rms 1.19 cm (ref:
1.17 cm).  Three physics switches in flight: z max diff 1.0 cm.
Parity runs: `PHYSICS=v2 bash setup/run_phase2.sh <name> <s> <scenario>`, then
`python scripts/compare_phase2.py <scenario> logs/gz_phase2_<name>.npz 0 1 v2`.

## Rendering
Everything is done with stock Gazebo (ogre2) features plus our own physics
plugin.  No external renderer, no downloaded assets.

| Element | How | Notes |
|---|---|---|
| Sky | `<sky>` in the world `<scene>` (ogre2 skybox shipped with gz-rendering) | Shown in the GUI and in camera sensors |
| Sun | Directional light ahead of the chase camera, so the water shows a sun glint; lower ambient light | All three worlds |
| Waves near the craft | `FoilDynamics` draws a 64 x 36 m triangle-list marker from the **same wave field the foils feel**, 20 Hz, on its own thread | Flat-shaded triangles (markers have per-face normals), so the sea looks slightly faceted up close |
| Far sea | Static plane with the same classic material as the wave patch, so there is no visible seam | Markers ignore PBR/environment maps, so neither surface uses them |
| Boat | Meshes from `scripts/make_craft_visuals.py`: hull with sheer stripe, non-skid deck, 8 textured solar panels on stand-offs, pilot, NACA-section struts, elliptic foils, pod and 3-blade propeller; PBR materials | Foils, struts, propeller: thesis Table A.1. Hull, deck, solar array, pilot: **illustrative**. Visual only: no extra mass, links or collisions |
| Spray | `gz-sim-particle-emitter-system`, one emitter per strut at the design waterline | **Cosmetic**: fixed height, does not react to waves or speed |

Water look parameters live in the `FoilDynamics` block of `model.sdf`:
`water_color`, `water_specular`, `water_alpha`, `wave_visual_cell`,
`wave_visual_half_x`, `wave_visual_half_y`, `wave_visual_rate`.

The drawing thread keeps the physics at real time: RTF 1.00, odometry 98 Hz
in an Hs 0.4 sea with the GUI open.  Drawing in the physics step had given
RTF dips to 0.05.  Calm-tour parity after the rendering work: z max diff
1.9 cm, rms 0.21 cm.

Pictures: `figures/render/before.png`, `after_gui.png`, `after_cam_irregular.png`.
Re-render: `bash setup/render_probe.sh <tag> [world] ["hf sea args"]`.

Not in Gazebo yet (later phases): moving foil visuals, hull buoyancy and
takeoff/landing, sensor noise + estimator.
