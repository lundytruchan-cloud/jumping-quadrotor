"""Tests for the ``evaluate_data.m`` pipeline port and validation."""

from pathlib import Path

import numpy as np
import pytest

from hopcopter_model.evaluate import central_diff, threshold_crossings

DATA_MAT = Path(__file__).resolve().parents[1] / "data/zenodo/StanceDynamics/StanceDynamics/data/20220502_140314.mat"
SAVED_MAT = Path(__file__).resolve().parents[1] / "data/zenodo/StanceDynamics/StanceDynamics/data_output/saved_data.mat"

requires_data = pytest.mark.skipif(
    not DATA_MAT.exists() or not SAVED_MAT.exists(),
    reason="Zenodo StanceDynamics data not downloaded",
)


def test_central_diff():
    t = np.linspace(0, 1, 11)
    x = t**2
    d = central_diff(t, x)
    assert d[5] == pytest.approx(2 * t[5], abs=1e-12)
    assert d[0] == d[1]
    assert d[-1] == d[-2]


def test_threshold_crossings():
    t = np.array([0.0, 1.0, 2.0, 3.0])
    z = np.array([0.10, 0.02, 0.05, 0.00])
    tc = threshold_crossings(t, z, 0.03)
    assert len(tc) == 3
    assert tc[0] == pytest.approx(1.0 - (0.03 - 0.02) / (0.10 - 0.02))
    assert tc[1] == pytest.approx(1.0 + (0.03 - 0.02) / (0.05 - 0.02))
    assert tc[2] == pytest.approx(2.0 + (0.03 - 0.05) / (0.00 - 0.05))


@requires_data
def test_pipeline_matches_authors_saved_data():
    import scipy.io

    from hopcopter_model.evaluate import analyze_jumps, load_mocap

    jumps = analyze_jumps(load_mocap(DATA_MAT), window=(26.0, 61.0))
    assert len(jumps) == 57
    saved = scipy.io.loadmat(SAVED_MAT)
    for key, atol in [
        ("landing_speed", 1e-6),
        ("theta_landing", 0.05),
        ("theta_takeoff", 0.05),
        ("veloc_takeoff", 0.05),
    ]:
        assert np.allclose(jumps[key], saved[key].ravel(), atol=atol), key


@requires_data
def test_validation_rmse_below_2_deg():
    from hopcopter_model.evaluate import analyze_jumps, load_mocap, rmse
    from hopcopter_model.params import PAPER_PARAMS

    jumps = analyze_jumps(load_mocap(DATA_MAT), PAPER_PARAMS, window=(26.0, 61.0))
    assert rmse(jumps, "delta_psi", "model_delta_psi") < 2.0
    assert rmse(jumps, "theta_TO", "model_theta_TO") < 2.0


@requires_data
def test_window_matches_author_selection_count():
    from hopcopter_model.evaluate import analyze_jumps, load_mocap

    full = analyze_jumps(load_mocap(DATA_MAT))
    selected = analyze_jumps(load_mocap(DATA_MAT), window=(26.0, 61.0))
    assert len(full) == 132
    assert len(selected) == 57
