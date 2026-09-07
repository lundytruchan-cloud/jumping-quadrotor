# Copyright 2026 Larry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the cascaded attitude PID controller."""

import math

import pytest

from quadcopter_description.attitude_controller import AttitudePidController
from quadcopter_description.attitude_math import (
    attitude_error,
    quaternion_from_rpy,
    quaternion_multiply,
    quaternion_normalize,
)


def test_identity_gives_zero_torque():
    pid = AttitudePidController()
    q = (1.0, 0.0, 0.0, 0.0)
    tau = pid.update(q, q, (0.0, 0.0, 0.0), 0.01)
    assert tau == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_roll_error_produces_restoring_torque():
    pid = AttitudePidController()
    q_ref = (1.0, 0.0, 0.0, 0.0)
    q_cur = quaternion_from_rpy(0.2, 0.0, 0.0)
    tau = pid.update(q_ref, q_cur, (0.0, 0.0, 0.0), 0.01)
    assert tau[0] < 0.0
    assert tau[1] == pytest.approx(0.0, abs=1e-12)
    assert tau[2] == pytest.approx(0.0, abs=1e-12)


def test_yaw_error_produces_restoring_torque():
    pid = AttitudePidController()
    q_ref = (1.0, 0.0, 0.0, 0.0)
    q_cur = quaternion_from_rpy(0.0, 0.0, 0.2)
    tau = pid.update(q_ref, q_cur, (0.0, 0.0, 0.0), 0.01)
    assert tau[0] == pytest.approx(0.0, abs=1e-12)
    assert tau[1] == pytest.approx(0.0, abs=1e-12)
    assert tau[2] < 0.0


def test_angular_rate_damping():
    pid = AttitudePidController()
    q = (1.0, 0.0, 0.0, 0.0)
    tau = pid.update(q, q, (2.0, 0.0, 0.0), 0.01)
    assert tau[0] < 0.0


def test_integral_anti_windup():
    pid = AttitudePidController(ki_rate=0.001, integral_limit=1e-4)
    q_ref = (1.0, 0.0, 0.0, 0.0)
    q_cur = quaternion_from_rpy(0.5, 0.0, 0.0)
    for _ in range(10000):
        pid.update(q_ref, q_cur, (0.0, 0.0, 0.0), 0.01)
    assert abs(pid.integral[0]) <= 1e-4 + 1e-9


def test_torque_clamped_to_max():
    pid = AttitudePidController(max_torque=0.05)
    q_ref = (1.0, 0.0, 0.0, 0.0)
    q_cur = quaternion_from_rpy(3.0, 0.0, 0.0)
    tau = pid.update(q_ref, q_cur, (0.0, 0.0, 0.0), 0.01)
    assert all(abs(t) <= 0.05 + 1e-9 for t in tau)


def _exp_quaternion(axis, angle):
    """Quaternion for a small body-frame rotation (w, x, y, z)."""
    if angle == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    half = 0.5 * angle
    return (math.cos(half),
            axis[0] / angle * math.sin(half),
            axis[1] / angle * math.sin(half),
            axis[2] / angle * math.sin(half))


def test_cascaded_pid_converges_in_simulated_rigid_body():
    """Integrate a simple rigid body (no gravity) with the PID for 5 s."""
    inertia = (1.45e-5, 1.45e-5, 2.67e-5)
    pid = AttitudePidController(
        kp_att=25.0, kp_rate=5.0e-4, ki_rate=0.0,
        max_torque=0.05)
    q = (1.0, 0.0, 0.0, 0.0)
    omega = [0.0, 0.0, 0.0]
    q_ref = quaternion_from_rpy(0.25, -0.15, 0.4)
    dt = 0.01
    for _ in range(500):
        tau = pid.update(q_ref, q, omega, dt)
        omega = [
            omega[i] + tau[i] / inertia[i] * dt for i in range(3)]
        rotation = [omega[i] * dt for i in range(3)]
        dq = _exp_quaternion(
            rotation,
            math.sqrt(sum(c * c for c in rotation)))
        q = quaternion_normalize(quaternion_multiply(q, dq))
    error = attitude_error(q_ref, q)
    assert math.sqrt(sum(c * c for c in error)) < 0.02
