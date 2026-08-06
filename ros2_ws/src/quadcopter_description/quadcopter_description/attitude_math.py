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

"""Quaternion helpers for the attitude controller (w, x, y, z order)."""

import math


def quaternion_multiply(q1, q2):
    """Hamilton product of two unit quaternions (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quaternion_conjugate(q):
    """Conjugate (inverse for unit quaternions)."""
    w, x, y, z = q
    return (w, -x, -y, -z)


def quaternion_normalize(q):
    """Normalize to unit norm; degenerate input falls back to identity."""
    norm = math.sqrt(sum(c * c for c in q))
    if norm == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / norm for c in q)


def quaternion_from_rpy(roll, pitch, yaw):
    """Quaternion (w, x, y, z) from intrinsic ZYX Euler angles."""
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quaternion_to_rpy(q):
    """Intrinsic ZYX Euler angles (roll, pitch, yaw) from a quaternion."""
    w, x, y, z = quaternion_normalize(q)
    sin_r_cos_p = 2.0 * (w * x + y * z)
    cos_r_cos_p = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_r_cos_p, cos_r_cos_p)
    sin_p = 2.0 * (w * y - z * x)
    if abs(sin_p) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_p)
    else:
        pitch = math.asin(sin_p)
    sin_y_cos_p = 2.0 * (w * z + x * y)
    cos_y_cos_p = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_y_cos_p, cos_y_cos_p)
    return (roll, pitch, yaw)


def rotation_vector(q):
    """Axis-angle representation: angle [rad] times the unit axis."""
    q = quaternion_normalize(q)
    w, x, y, z = q
    vec_norm = math.sqrt(x * x + y * y + z * z)
    if vec_norm < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vec_norm, w)
    scale = angle / vec_norm
    return (x * scale, y * scale, z * scale)


def attitude_error(q_ref, q_cur):
    """
    Body-frame rotation vector taking the current attitude to the setpoint.

    q_err = q_cur^-1 * q_ref, shortest-path sign chosen; the returned vector
    is small for small attitude errors and points in the direction the body
    must rotate about to reach the reference.
    """
    q_err = quaternion_multiply(quaternion_conjugate(q_cur), q_ref)
    if q_err[0] < 0.0:
        q_err = tuple(-c for c in q_err)
    return rotation_vector(q_err)
