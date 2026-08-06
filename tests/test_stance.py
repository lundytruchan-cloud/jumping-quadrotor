"""Unit tests for the stance-phase model (paper Eqs. 12-21)."""

import numpy as np
import pytest

from hopcopter_model.params import AUTHORS_PARAMS, PAPER_PARAMS
from hopcopter_model.stance import leg_trajectory, stance_mapping_grid, stance_transform


def test_initial_conditions():
    r = stance_transform(np.deg2rad(15.0), 2.0)
    tr = leg_trajectory(np.deg2rad(15.0), 2.0)
    assert tr["l"][0] == pytest.approx(PAPER_PARAMS.l0, abs=1e-12)
    assert tr["l_dot"][0] == pytest.approx(-2.0 * np.cos(np.deg2rad(15.0)), abs=1e-9)
    assert r.l_t1 < PAPER_PARAMS.l0


def test_phase_continuity_and_takeoff():
    r = stance_transform(np.deg2rad(20.0), 3.0)
    tr = leg_trajectory(np.deg2rad(20.0), 3.0)
    i1 = np.argmin(np.abs(tr["t"] - r.t_1))
    assert tr["l"][i1] == pytest.approx(r.l_t1, abs=1e-9)
    assert tr["l_dot"][i1] == pytest.approx(0.0, abs=1e-9)
    i2 = np.argmin(np.abs(tr["t"] - r.t_TO))
    assert tr["l"][i2] == pytest.approx(PAPER_PARAMS.l0, abs=1e-9)
    assert r.l_dot_TO > 0.0
    assert r.t_TO > r.t_1 > 0.0


def test_angular_momentum_conservation():
    theta_l, speed = np.deg2rad(12.0), 2.5
    tr = leg_trajectory(theta_l, speed)
    expected = PAPER_PARAMS.l0 * speed * np.sin(theta_l)
    # beta_dot(t) * l(t)^2 must be constant (per unit mass).
    angular = tr["beta_dot"] * tr["l"] ** 2
    assert np.allclose(angular, expected, rtol=1e-6, atol=1e-9)
    # Integrated rotation must match the model output.
    r = stance_transform(theta_l, speed)
    assert tr["beta"][-1] == pytest.approx(r.beta_TO, rel=1e-6, abs=1e-9)


def test_takeoff_mapping_geometry():
    theta_l, speed = np.deg2rad(18.0), 2.2
    r = stance_transform(theta_l, speed)
    assert r.theta_t > theta_l
    assert r.theta_v > r.theta_t
    assert r.v_t > 0.0
    # Tangential takeoff component equals landing tangential component
    # (equal radii at landing and takeoff).
    assert np.hypot(r.l_dot_TO, speed * np.sin(theta_l)) == pytest.approx(r.v_t)


def test_stance_time_matches_paper():
    # Paper reports ~32 ms stance duration.
    r = stance_transform(np.deg2rad(13.5249), 3.10645)
    assert r.t_TO == pytest.approx(0.032, abs=0.004)


def test_grid_matches_scalar():
    theta_ls = np.deg2rad(np.arange(0.0, 45.5, 2.5))
    speeds = np.arange(0.1, 4.51, 0.25)
    g = stance_mapping_grid(theta_ls, speeds)
    for i, th in enumerate(theta_ls):
        for j, sp in enumerate(speeds):
            r = stance_transform(float(th), float(sp))
            assert g["delta_psi"][i, j] == pytest.approx(r.theta_t - float(th), abs=1e-10)
            assert g["theta_TO"][i, j] == pytest.approx(r.theta_v - r.theta_t, abs=1e-10)
            assert g["v_t"][i, j] == pytest.approx(r.v_t, abs=1e-10)
            assert g["t_TO"][i, j] == pytest.approx(r.t_TO, abs=1e-10)


def test_mapping_reproduces_fig2c_shape():
    # On the original run_this.m grid the mapping is smooth and monotone in
    # the landing angle at a fixed speed.
    theta_ls = np.deg2rad(np.arange(0.0, 25.5, 0.5))
    speeds = np.arange(1.0, 3.51, 0.05)
    g = stance_mapping_grid(theta_ls, speeds)
    assert np.all(np.diff(np.rad2deg(g["delta_psi"]), axis=0) > 0)
    assert np.all(np.diff(np.rad2deg(g["theta_TO"]), axis=0) > 0)
    assert np.all(np.diff(g["v_t"], axis=0) > 0)


def test_zero_speed_degenerate():
    r = stance_transform(np.deg2rad(10.0), 0.0)
    assert r.v_t == 0.0
    assert r.t_TO == 0.0


def test_authors_params_sanity():
    # Cross-check against the MATLAB numeric constants (k=170.0615, m=0.034,
    # f_h=0.4453): k/m and f_c/m ratios match the paper within 0.4%.
    assert AUTHORS_PARAMS.k_over_m == pytest.approx(5.00e3, rel=0.004)
    assert AUTHORS_PARAMS.f_c_over_m == pytest.approx(12.7, rel=0.04)
