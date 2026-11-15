"""Compare the Python port's trim point and linear model against the values
published in the original repo / thesis (Appendix A.3, Sec. 3.4.1)."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hydrofoil.plant import Hydrofoil
from hydrofoil.linearize import trim, linearize, reduce, LIN_NAMES

np.set_printoptions(precision=4, suppress=True, linewidth=140)
# Thesis Eq. (A.3.1)/(A.3.2) MAGNITUDES (minus signs are lost in the PDF text layer)
A_thesis = np.array([
    [0, 0, 7.0, 0, 1.0, 0, 0],
    [0, 0, 0, 0, 0, 1.0, 0],
    [0, 0, 0, 0, 0, 0, 1.0],
    [0, 0, 9.81, 1.179, 0.8501, 0, 0.02685],
    [0, 0, 0, 2.803, 12.86, 0, 3.894],
    [0, 0, 0, 0, 0, 64.36, 0],
    [0, 0, 0, 0.4837, 0.02095, 0, 15.44]])
B_thesis = np.array([
    [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0],
    [1.365, 1.365, 1.129, 0.007056],
    [30.15, 30.15, 29.41, 0],
    [129.6, 129.6, 0, 0],
    [13.91, 13.91, 31.1, 0.002895]])

m = Hydrofoil(wagner=False)
core, delta, rpm = trim(m, 7.0)
print("TRIM @ 7 m/s   port:   dF = %.5f rad (%.3f deg)  dA = %.5f rad (%.3f deg)  rpm = %.1f"
      % (delta[0], np.rad2deg(delta[0]), delta[2], np.rad2deg(delta[2]), rpm))
print("               repo:   dF = 0.05740 rad            dA = 0.05205 rad             rpm = 1169.5 (reftrack.m rpm_const)")
A, B = linearize(m, core, delta, rpm)
print("\nA (port), states", LIN_NAMES); print(A)
print("\n|A| thesis"); print(A_thesis)
print("\nB (port), inputs [dFS dFP dA rpm]"); print(B)
print("\n|B| thesis"); print(B_thesis)
nz = A_thesis > 0
print("\nratio |A_port|/|A_thesis| on non-zero entries:")
for (i, j) in zip(*np.nonzero(nz)):
    print(f"  A[{LIN_NAMES[i]:>5},{LIN_NAMES[j]:>5}]  port {A[i,j]:+9.4f}  thesis {A_thesis[i,j]:8.4f}  ratio {abs(A[i,j])/A_thesis[i,j]:.3f}")
for (i, j) in zip(*np.nonzero(B_thesis > 0)):
    print(f"  B[{LIN_NAMES[i]:>5},{j}]  port {B[i,j]:+9.4f}  thesis {B_thesis[i,j]:8.4f}  ratio {abs(B[i,j])/B_thesis[i,j]:.3f}")
ev = np.sort(np.linalg.eigvals(A).real)
print("\npoles full  port  ", ev)
print("poles full  thesis [-64.358 -15.489 -12.572 -1.615 0 0 0.196]")
Ar, Br = reduce(A, B)
print("poles reduced port ", np.sort(np.linalg.eigvals(Ar).real))
print("poles reduced thesis [-64.358 -15.489 -12.572 0 0 0]")
