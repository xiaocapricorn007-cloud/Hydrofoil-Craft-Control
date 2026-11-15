# Replication report: Python port of jskoun/MPC_HydrofoilControl

This is a port of the MATLAB/Simulink code behind I. Skounakis-Kounavis, *Dynamic Modeling & Predictive Control of a Hydrofoil Vessel* (diploma thesis, NTUA, 2025). The original repo is github.com/jskoun/MPC_HydrofoilControl.

The original needs Simulink, the Control System Toolbox and the MPC Toolbox. This port needs only numpy, scipy, osqp and matplotlib.

**What it is:** a 215 kg solar-class (Monaco Energy Boat Challenge) boat on fully submerged foils. It has two fore foils and one aft foil, all fully rotating. The craft runs at 7 m/s with the CG 0.3 m above the water. Height, roll and pitch are controlled at 10 Hz.

## How the physics was recovered
The repo's `.m` files contain only parameters and controller design; the plant itself lives in Simulink.

`.slx` files are zip archives of XML. The block wiring, lookup tables and MATLAB Function code were read directly from `hydrofoil_sim_unified.slx` and `simplified_sim_for_linearize.slx`, and `hydrofoil/plant.py` reproduces them block by block:

- **Foil forces:** XFLR5 polar lookups for the NACA 63-412 elliptic wing.
- **Free-surface effects:** lift loss (thesis eq. 2.5.17) and the induced-drag increase.
- **Extra drag terms:** wave drag and interference drag.
- **Unsteady lift:** a Wagner/Jones filter on every foil in the full simulation; the linearization model has none.
- **Struts:** drag, plus their pitch and roll moments.
- **Roll damping:** span integrals.
- **Propulsion:** Wageningen-B propeller K_T fit, with a speed PI (P = 250, I = 200 around 1200 rpm).
- **Actuators:** first-order lag with τ = 0.1 s.

The Simulink gain `actuation_tau` is never defined anywhere in the repo, so the thesis value τ = 0.1 s is used.

## Validation against the thesis
`scripts/validate_linear.py`, `tests/`

| quantity | port | thesis / repo |
|---|---|---|
| trim, fore foils | 3.290° | 0.0574 rad = 3.289° |
| trim, aft foil | 2.985° | 0.05205 rad = 2.982° |
| trim rpm, 7 m/s | 1170.4 | 1169.5 |
| Z_w/m, Z_u/m, Z_q | −12.86, −2.803, 3.894 | 12.86, 2.803, 3.894 |
| M_q/I_y | −15.43 | 15.44 |
| B: heave, roll, pitch per fore foil | −30.15, ∓129.6, 13.91 | 30.15, 129.6, 13.91 |
| B: heave, pitch for the aft foil | −29.41, −31.09 | 29.41, 31.1 |
| roll damping K_p/I_x | −66.2 | 64.36 (3% off) |
| poles (full model) | −66.2, −15.5, −12.6, −1.66, ≈0, 0.116 (double) | −64.4, −15.5, −12.6, −1.62, 0, 0, 0.196 |

The minus signs in the thesis column are missing because the PDF text layer drops them.

The Simulink linearization's A matrix has an all-zero ∂/∂z column, meaning the free-surface height sensitivity is dropped. The nonlinear plant keeps it. For a faithful replication, controllers here are designed on the same zero-column model (`build_models(thesis_consistent=True)`).

## Benchmark: +0.2 m ride-height step at 7 m/s
`scripts/step_benchmark.py`; compare thesis Tables 5.3–5.5.

| controller | ISE | IAE | ITAE | effort | thesis ISE / IAE / ITAE / effort |
|---|---|---|---|---|---|
| LQI | 1.271 | 1.540 | 1.961 | 2.042 | 1.310 / 1.577 / 1.994 / 1.937 |
| LQI-S | 1.406 | 1.659 | 2.200 | 1.463 | 1.455 / 1.693 / 2.218 / 1.416 |
| LQI-SW | 1.206 | 1.355 | 1.634 | 1.281 | 1.284 / 1.410 / 1.682 / 1.239 |
| MPC-I | 1.258 | 1.146 | 1.204 | 1.798 | 1.317 / 1.058 / 0.932 / 1.540 |
| MPC-ID | 1.147 | 0.748 | 0.548 | 1.027 | 1.273 / 0.851 / 0.612 / 0.928 |
| MPC-IP | 0.259 | 0.715 | 0.705 | 1.174 | 0.130 / 0.447 / 0.319 / 1.073 |
| MPC-IDP | 0.139 | 0.295 | 0.163 | 0.665 | 0.140 / 0.292 / 0.128 / 0.647 |

The thesis does not state the step time; t = 0.5 s is used here. ITAE depends directly on that choice.

## Known deviations
- **State estimator:** the MPC Toolbox's built-in estimator is replaced by a steady-state Kalman filter on the same augmented model, with the same input-disturbance gains (0.4 and 1.5). This is the most likely reason MPC-IP is about 2× off on ISE.
- **Preview alignment:** the preview rows start at the current sample. This is an inference: it is what reproduces Table 5.5, but the previewer block's internals are not in the repo.
- **Broaching:** when a foil leaves the water its forces are set to 0. The original formulas are undefined there.
- **Integration:** fixed-step RK4 at 5 ms, where Simulink used a variable-step solver.
- **Waves:** an independent Airy-wave implementation, not the MSS Toolbox code.

## Other results
- **Wave sweep** (`figures/wave_response.png`): regular waves, ζa = 0.1 m, head and following seas. Same scenario as thesis Fig. 5.8; that figure has no numeric table to compare against.
- **Speed ramp** (`figures/speed_ramp.png`): 7→10 m/s with a 0.1 m height change, the `reftrack.m` scenario. All controllers survive it.
  - MPC-IDP had ISE 0.002; LPV-MPC-IDP had ISE 0.008.
  - Gain scheduling did not help in this scenario.
