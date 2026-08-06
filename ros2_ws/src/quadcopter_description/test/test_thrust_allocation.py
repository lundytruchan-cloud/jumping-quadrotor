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

"""Unit tests for the attitude-priority thrust allocation (paper Eq. 34-35)."""

import math

import pytest

from quadcopter_description.thrust_allocation import ThrustAllocation

ARM = 0.032527
MOMENT_RATIO = 0.006
G = 9.81
MASS = 0.0348


def torque_matrix(allocator):
    """3x4 torque mixing matrix matching the allocator geometry."""
    a = allocator.arm
    c = allocator.moment_ratio
    return [
        [a, a, -a, -a],
        [-a, a, a, -a],
        [-c, c, -c, c],
    ]


def apply_torque_matrix(matrix, f):
    return tuple(sum(matrix[i][j] * f[j] for j in range(4))
                 for i in range(3))


def test_zero_command_gives_zero_thrust():
    allocator = ThrustAllocation()
    result = allocator.allocate(0.0, (0.0, 0.0, 0.0))
    assert result['thrust'] == pytest.approx(0.0, abs=1e-12)
    assert result['f'] == pytest.approx((0.0, 0.0, 0.0, 0.0), abs=1e-12)


def test_hover_thrust_is_shared_equally():
    allocator = ThrustAllocation()
    result = allocator.allocate(MASS * G, (0.0, 0.0, 0.0))
    expected = MASS * G / 4.0
    assert result['f'] == pytest.approx(
        (expected, expected, expected, expected), abs=1e-12)
    assert result['thrust'] == pytest.approx(MASS * G, abs=1e-9)


@pytest.mark.parametrize('torque', [
    (1.0e-4, -2.0e-4, 5.0e-5),
    (-3.0e-4, 0.0, 0.0),
    (0.0, 2.0e-4, -1.0e-4),
])
def test_torque_is_reproduced_exactly(torque):
    allocator = ThrustAllocation()
    result = allocator.allocate(0.2, torque)
    matrix = torque_matrix(allocator)
    actual = apply_torque_matrix(matrix, result['f'])
    assert actual == pytest.approx(torque, abs=1e-12)
    assert all(f >= -1e-12 for f in result['f'])


def test_attitude_priority_raises_thrust_when_desired_too_low():
    allocator = ThrustAllocation()
    # Pure pitch torque: T_min = |tau_y| / arm; requested thrust 0 is
    # infeasible, so the allocator must return the smallest feasible T.
    torque_y = 3.0e-4
    result = allocator.allocate(0.0, (0.0, torque_y, 0.0))
    t_min = torque_y / ARM
    assert result['thrust'] == pytest.approx(t_min, abs=1e-9)
    assert min(result['f']) == pytest.approx(0.0, abs=1e-9)
    matrix = torque_matrix(allocator)
    assert apply_torque_matrix(matrix, result['f']) == pytest.approx(
        (0.0, torque_y, 0.0), abs=1e-12)


def test_attitude_priority_keeps_desired_thrust_when_feasible():
    allocator = ThrustAllocation()
    result = allocator.allocate(MASS * G, (0.0, 1.0e-4, 0.0))
    assert result['thrust'] == pytest.approx(MASS * G, abs=1e-9)
    assert all(f >= -1e-12 for f in result['f'])


def test_min_thrust_property_for_random_torques():
    allocator = ThrustAllocation()
    matrix = torque_matrix(allocator)
    for _ in range(50):
        torque = (
            (hash((0, _)) % 7 - 3) * 5.0e-5,
            (hash((1, _)) % 7 - 3) * 5.0e-5,
            (hash((2, _)) % 7 - 3) * 2.0e-5,
        )
        result = allocator.allocate(-1.0, torque)
        assert result['thrust'] >= -1e-9
        assert all(f >= -1e-9 for f in result['f'])
        assert apply_torque_matrix(matrix, result['f']) == pytest.approx(
            torque, abs=1e-12)
        # No smaller feasible total thrust exists: reduce T a tiny bit and a
        # motor must go negative.
        reduced = [
            result['f'][i] - result['thrust'] / 4.0
            + (result['thrust'] - 1e-9) / 4.0
            for i in range(4)
        ]
        assert min(reduced) < -1e-12


def test_speeds_from_thrust():
    allocator = ThrustAllocation()
    f = (0.0854, 0.0854, 0.0854, 0.0854)
    speeds = allocator.speeds_from_thrust(f, motor_constant=3.0e-8,
                                          max_speed=2200.0)
    expected = math.sqrt(0.0854 / 3.0e-8)
    assert speeds == pytest.approx([expected] * 4, abs=1e-6)


def test_speeds_clamp_to_max_and_floor_at_zero():
    allocator = ThrustAllocation()
    speeds = allocator.speeds_from_thrust(
        (-0.1, 0.0, 1.0, 0.5), motor_constant=3.0e-8, max_speed=2200.0)
    assert speeds[0] == 0.0
    assert speeds[1] == 0.0
    assert speeds[2] == pytest.approx(2200.0)
    assert speeds[3] > 0.0
