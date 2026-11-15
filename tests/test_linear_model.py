"""Regression: the port's trim and linear model must match the thesis
(Appendix A.3, magnitudes) and the repo's trim constants."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np
from hydrofoil.plant import Hydrofoil
from hydrofoil.linearize import trim, linearize


def test_trim_matches_repo():
    core, d, rpm = trim(Hydrofoil(wagner=False), 7.0)
    assert abs(d[0] - 0.0574) < 2e-4 and abs(d[2] - 0.05205) < 2e-4
    assert abs(rpm - 1169.5) < 5


def test_key_derivatives_match_thesis():
    m = Hydrofoil(wagner=False)
    A, B = linearize(m, *trim(m, 7.0))
    # [z, phi, theta, u, w, p, q] ; inputs [FS, FP, A, rpm]
    pairs = [(A[4, 4], -12.86), (A[4, 3], -2.803), (A[4, 6], 3.894), (A[6, 6], -15.44),
             (A[6, 3], -0.4837), (A[3, 3], -1.179), (B[4, 0], -30.15), (B[4, 2], -29.41),
             (B[5, 0], -129.6), (B[6, 0], 13.91), (B[6, 2], -31.1), (A[5, 5], -64.36)]
    for got, want in pairs:
        assert abs(got - want) / abs(want) < 0.035, (got, want)
