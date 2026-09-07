"""Unit tests for the high-level jump planner (paper Eqs. 22-33).

The planner combines the ballistic aerial-phase prediction, the task-4
stance mapping and the iterative landing-attitude solver with the
no-slip constraint and the height-control timing laws.
"""

import numpy as np
import pytest

from hopcopter_model.jump_planner import (
    ballistic_flight_duration,
    circle_reference,
    desired_takeoff_velocity,
    no_slip_angle,
    no_slip_ok,
    powered_ascent_duration,
    predict_landing_from_apex,
    predict_landing_from_takeoff,
    solve_landing_attitude,
    stance_landing_to_takeoff,
    step_reference,
)
from hopcopter_model.params import PAPER_PARAMS

G = 9.80665


def test_predict_landing_from_apex_ballistic():
    p_apex = np.array([0.0, 0.0, 1.0])
    v_apex = np.array([0.3, 0.0, 0.0])
    z_b = np.array([0.0, 0.0, 1.0])
    pred = predict_landing_from_apex(p_apex, v_apex, z_b, PAPER_PARAMS.l0, G)
    t_expected = np.sqrt(2.0 * (1.0 - PAPER_PARAMS.l0) / G)
    assert pred.t_fall == pytest.approx(t_expected, rel=1e-9)
    assert pred.p_landing[0] == pytest.approx(0.3 * t_expected, rel=1e-9)
    assert pred.p_landing[1] == pytest.approx(0.0, abs=1e-12)
    assert pred.p_landing[2] == pytest.approx(PAPER_PARAMS.l0, abs=1e-9)
    assert pred.v_landing[0] == pytest.approx(0.3, abs=1e-12)
    assert pred.v_landing[2] == pytest.approx(-G * t_expected, rel=1e-9)


def test_predict_landing_from_takeoff_flat_ground():
    p_to = np.array([0.0, 0.0, PAPER_PARAMS.l0])
    v_to = np.array([0.5, 0.0, 3.0])
    z_b_next = np.array([0.0, 0.0, 1.0])
    pred = predict_landing_from_takeoff(p_to, v_to, z_b_next, PAPER_PARAMS.l0, G)
    t_expected = 2.0 * 3.0 / G
    assert pred.t_fall == pytest.approx(t_expected, rel=1e-9)
    assert pred.p_landing[0] == pytest.approx(0.5 * t_expected, rel=1e-9)
    assert pred.p_landing[2] == pytest.approx(PAPER_PARAMS.l0, abs=1e-9)


def test_desired_takeoff_velocity_deadbeat():
    p_ld = np.array([0.0, 0.0, 0.0])
    p_des = np.array([0.4, 0.0, 0.0])
    z_d = 0.5
    v = desired_takeoff_velocity(p_ld, p_des, z_d, G)
    v_z = np.sqrt(2.0 * G * z_d)
    assert v[2] == pytest.approx(v_z, rel=1e-12)
    assert v[0] == pytest.approx(G * 0.4 / (2.0 * v_z), rel=1e-12)
    assert v[1] == pytest.approx(0.0, abs=1e-12)


def test_powered_ascent_duration():
    # vz = 2 m/s -> nominal height 0.2039 m; z_d = 0.5 m needs a burst.
    d = powered_ascent_duration(2.0, 0.5, G)
    expected = 0.5 / 2.0 - 2.0 / (2.0 * G)
    assert d == pytest.approx(expected, rel=1e-12)
    # If the nominal height already exceeds the desired height: no thrust.
    assert powered_ascent_duration(2.0, 0.1, G) == pytest.approx(0.0, abs=1e-12)
    # Zero takeoff speed cannot gain height with hover-level thrust.
    assert powered_ascent_duration(0.0, 0.5, G) == pytest.approx(0.0, abs=1e-12)


def test_ballistic_flight_duration():
    d = ballistic_flight_duration(2.0, 0.5, G)
    expected = 2.0 / G + np.sqrt(2.0 * 0.5 / G)
    assert d == pytest.approx(expected, rel=1e-12)


def test_stance_landing_to_takeoff_vertical():
    v_ld = np.array([0.0, 0.0, -3.0])
    z_b = np.array([0.0, 0.0, 1.0])
    p_ld = np.array([0.0, 0.0, PAPER_PARAMS.l0])
    res = stance_landing_to_takeoff(p_ld, v_ld, z_b, PAPER_PARAMS)
    assert np.allclose(res.z_b_to, [0.0, 0.0, 1.0], atol=1e-12)
    assert np.allclose(res.v_to[:2], [0.0, 0.0], atol=1e-12)
    assert res.v_to[2] == pytest.approx(res.v_t, rel=1e-12)
    assert res.v_t > 0.0
    assert np.allclose(res.p_to, p_ld, atol=1e-12)


def test_stance_landing_to_takeoff_in_plane():
    theta = 0.25
    v_ld = np.array([1.0, 0.0, -3.0])
    z_b = np.array([np.sin(theta), 0.0, np.cos(theta)])
    p_ld = np.array([0.0, 0.0, PAPER_PARAMS.l0])
    res = stance_landing_to_takeoff(p_ld, v_ld, z_b, PAPER_PARAMS)
    a = -v_ld / np.linalg.norm(v_ld)
    n = np.cross(a, z_b)
    n = n / np.linalg.norm(n)
    # The takeoff state must stay in the landing plane (coplanar model).
    assert abs(np.dot(res.z_b_to, n)) < 1e-9
    assert abs(np.dot(res.v_to, n)) < 1e-9
    assert np.linalg.norm(res.z_b_to) == pytest.approx(1.0, abs=1e-12)
    assert np.linalg.norm(res.v_to) == pytest.approx(res.v_t, rel=1e-12)


def test_no_slip_constraint():
    assert no_slip_angle(np.array([0.0, 0.0, 1.0])) == pytest.approx(0.0, abs=1e-12)
    z = np.array([np.sin(np.deg2rad(30.0)), 0.0, np.cos(np.deg2rad(30.0))])
    assert no_slip_angle(z) == pytest.approx(np.deg2rad(30.0), rel=1e-9)
    assert no_slip_ok(z, mu=0.8)
    z2 = np.array([np.sin(np.deg2rad(50.0)), 0.0, np.cos(np.deg2rad(50.0))])
    assert not no_slip_ok(z2, mu=0.8)


def test_solve_landing_attitude_stationary():
    p_apex = np.array([0.0, 0.0, 0.9])
    v_apex = np.array([0.0, 0.0, 0.0])
    p_des = np.array([0.0, 0.0, 0.0])
    plan = solve_landing_attitude(
        p_apex, v_apex, p_des, z_d=0.6, params=PAPER_PARAMS, g=G, mu=0.8)
    assert plan.success
    assert no_slip_ok(plan.z_b_landing, mu=0.8)
    assert no_slip_ok(plan.z_b_takeoff, mu=0.8)
    assert np.linalg.norm(plan.z_b_landing[:2]) < 1e-3
    assert np.linalg.norm(plan.p_landing_next_pred[:2]) < 0.05


def test_solve_landing_attitude_moves_target():
    p_apex = np.array([0.0, 0.0, 0.9])
    v_apex = np.array([0.2, 0.0, 0.0])
    p_des = np.array([0.6, 0.0, 0.0])
    plan = solve_landing_attitude(
        p_apex, v_apex, p_des, z_d=0.6, params=PAPER_PARAMS, g=G, mu=0.8)
    assert plan.success
    assert no_slip_ok(plan.z_b_landing, mu=0.8)
    assert no_slip_ok(plan.z_b_takeoff, mu=0.8)
    assert np.linalg.norm(plan.p_landing_next_pred - p_des) < 0.3
    assert plan.z_b_landing[0] > 0.0  # tilts toward the target


def test_solve_landing_attitude_respects_tilt_limit():
    p_apex = np.array([0.0, 0.0, 0.9])
    v_apex = np.array([0.5, 0.0, 0.0])
    p_des = np.array([1.5, 0.0, 0.0])
    plan = solve_landing_attitude(
        p_apex, v_apex, p_des, z_d=0.6, params=PAPER_PARAMS, g=G, mu=0.8,
        tilt_limit=np.deg2rad(18.0))
    assert no_slip_angle(plan.z_b_landing) <= np.deg2rad(18.0) + 1e-6
    # The no-slip constraint (Eq. 32) still applies to the takeoff axis.
    assert no_slip_ok(plan.z_b_takeoff, mu=0.8)


def test_references():
    t = np.linspace(0.0, 4.0, 9)
    r = circle_reference(t, radius=0.5, omega=0.5)
    assert r.shape == (9, 3)
    assert np.allclose(np.hypot(r[:, 0], r[:, 1]), 0.5, atol=1e-12)
    assert np.allclose(r[:, 2], 0.0)
    s = step_reference(t, targets=[(0.0, 0.0), (0.6, 0.0)], hold_period=1.0)
    assert s.shape == (9, 3)
    assert np.allclose(s[0], [0.0, 0.0, 0.0])
    assert np.allclose(s[2], [0.6, 0.0, 0.0])
