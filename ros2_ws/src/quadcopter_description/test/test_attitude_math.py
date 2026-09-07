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

"""Unit tests for quaternion helpers used by the attitude controller."""

import math

import pytest

from quadcopter_description.attitude_math import (
    attitude_error,
    quaternion_conjugate,
    quaternion_from_rpy,
    quaternion_multiply,
    quaternion_normalize,
    quaternion_to_rpy,
    rotation_vector,
)


def norm(q):
    return math.sqrt(sum(c * c for c in q))


def test_multiply_identity():
    q = (0.5, 0.5, 0.5, 0.5)
    identity = (1.0, 0.0, 0.0, 0.0)
    assert quaternion_multiply(q, identity) == pytest.approx(q)
    assert quaternion_multiply(identity, q) == pytest.approx(q)


def test_conjugate_is_inverse():
    q = quaternion_normalize((0.7, 0.2, -0.4, 0.3))
    result = quaternion_multiply(q, quaternion_conjugate(q))
    assert result == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)


def test_from_rpy_zero_is_identity():
    assert quaternion_from_rpy(0.0, 0.0, 0.0) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_from_to_rpy_roundtrip():
    rpy = (0.3, -0.4, 0.8)
    q = quaternion_from_rpy(*rpy)
    assert norm(q) == pytest.approx(1.0)
    assert quaternion_to_rpy(q) == pytest.approx(rpy, abs=1e-9)


def test_rotation_vector_small_angle():
    # Roll +0.2 rad about x should give a rotation vector (+0.2, 0, 0).
    q = quaternion_from_rpy(0.2, 0.0, 0.0)
    assert rotation_vector(q) == pytest.approx((0.2, 0.0, 0.0), abs=1e-9)


def test_attitude_error_zero_when_attitudes_match():
    q = quaternion_from_rpy(0.3, -0.2, 0.5)
    assert attitude_error(q, q) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_attitude_error_roll_is_negative_for_extra_current_roll():
    # Current attitude has +0.3 rad roll, reference is level: the body-frame
    # error must point back towards level (negative x rotation).
    q_ref = (1.0, 0.0, 0.0, 0.0)
    q_cur = quaternion_from_rpy(0.3, 0.0, 0.0)
    assert attitude_error(q_ref, q_cur) == pytest.approx(
        (-0.3, 0.0, 0.0), abs=1e-9)


def test_attitude_error_yaw_uses_shortest_path():
    q_ref = quaternion_from_rpy(0.0, 0.0, -3.0)
    q_cur = quaternion_from_rpy(0.0, 0.0, 3.0)
    error = attitude_error(q_ref, q_cur)
    assert error[0] == pytest.approx(0.0, abs=1e-9)
    assert error[1] == pytest.approx(0.0, abs=1e-9)
    assert error[2] == pytest.approx(2.0 * math.pi - 6.0, abs=1e-6)


def test_normalize():
    q = (2.0, 2.0, 2.0, 2.0)
    assert norm(quaternion_normalize(q)) == pytest.approx(1.0)
