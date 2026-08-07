"""Unit tests for the aerodynamic stabilizer (paper Eq. S33, flat-plate
theory) and its activation/control logic (task 8).
"""

import numpy as np
import pytest

from hopcopter_model.stabilizer import (
    AERO_PARAMS,
    AeroStabilizerParams,
    aero_aligning_torque,
    descent_control_mode,
    stabilizer_active,
)


def test_torque_aligns_body_axis_with_opposite_velocity():
    # Falling with a forward velocity component while the body is tilted
    # toward +x: the torque must rotate z_b toward -v_hat.
    v = np.array([1.0, 0.0, -3.0])
    z_b = np.array([np.sin(np.deg2rad(10.0)), 0.0, np.cos(np.deg2rad(10.0))])
    out = aero_aligning_torque(z_b, v, AERO_PARAMS)
    assert out.torque_norm > 0.0
    # z_b_dot = z_b x tau (unit scale): its perpendicular component must
    # point toward -v_hat.
    a = -v / np.linalg.norm(v)
    # Rotational dynamics: z_b_dot ~ tau x z_b (omega ~ tau).
    z_dot = np.cross(out.torque, z_b)
    a_perp = a - np.dot(a, z_b) * z_b
    assert np.dot(z_dot, a_perp) > 0.0


def test_torque_zero_when_body_aligned_with_velocity():
    v = np.array([1.0, 0.0, -3.0])
    z_b = -v / np.linalg.norm(v)
    out = aero_aligning_torque(z_b, v, AERO_PARAMS)
    assert out.torque_norm == pytest.approx(0.0, abs=1e-12)


def test_torque_zero_when_antiparallel_or_still():
    v = np.array([1.0, 0.0, -3.0])
    z_b = v / np.linalg.norm(v)
    assert aero_aligning_torque(z_b, v, AERO_PARAMS).torque_norm == 0.0
    assert aero_aligning_torque(np.array([0.0, 0.0, 1.0]),
                                np.zeros(3), AERO_PARAMS).torque_norm == 0.0


def test_force_matches_flat_plate_magnitude():
    # Paper SM Eq. S33: F_A = rho * S * sin(alpha) * U^2.
    params = AeroStabilizerParams(
        rho=1.2, s_eff=2.0 * 39.0e-4, k_eff=1.0, moment_arm=0.06)
    v = np.array([0.0, 0.0, -4.4])
    z_b = np.array([np.sin(np.deg2rad(10.0)), 0.0, np.cos(np.deg2rad(10.0))])
    out = aero_aligning_torque(z_b, v, params)
    expected_force = (
        1.2 * 2.0 * 39.0e-4 * np.sin(np.deg2rad(10.0)) * 4.4 ** 2)
    assert out.force_mag == pytest.approx(expected_force, rel=1e-9)
    # The torque is the force times the effective moment arm.
    assert out.torque_norm == pytest.approx(
        params.moment_arm * expected_force, rel=1e-9)
    # Order of magnitude of the paper estimate (FA ~= 10 mN at 4.4 m/s).
    assert 5.0e-3 < out.force_mag < 0.1


def test_torque_scales_quadratically_with_speed():
    v1 = np.array([0.0, 0.0, -3.0])
    v2 = np.array([0.0, 0.0, -6.0])
    z_b = np.array([0.3, 0.0, np.sqrt(1.0 - 0.09)])
    t1 = aero_aligning_torque(z_b, v1, AERO_PARAMS).torque_norm
    t2 = aero_aligning_torque(z_b, v2, AERO_PARAMS).torque_norm
    assert t2 == pytest.approx(4.0 * t1, rel=1e-9)


@pytest.mark.parametrize("strategy,state,expected", [
    ("attitude_only", "DESCENT", False),
    ("attitude_only", "ASCENT_POWERED", False),
    ("stabilizer_only", "DESCENT", True),
    ("stabilizer_only", "STANDBY", True),
    ("combined", "DESCENT", True),
    ("combined", "PRE_LAND", True),
    ("combined", "STANCE", True),
    ("combined", "ASCENT_POWERED", False),
    ("combined", "ASCENT_BALLISTIC", False),
    ("combined", "STANDBY", False),
])
def test_stabilizer_activation_schedule(strategy, state, expected):
    assert stabilizer_active(strategy, state) is expected


@pytest.mark.parametrize("strategy,state,expected", [
    ("attitude_only", "DESCENT", "attitude_only"),
    ("stabilizer_only", "DESCENT", "zero_thrust"),
    ("stabilizer_only", "ASCENT_POWERED", "hover"),
    ("combined", "PRE_LAND", "attitude_only"),
])
def test_descent_control_mode(strategy, state, expected):
    assert descent_control_mode(strategy, state) == expected
