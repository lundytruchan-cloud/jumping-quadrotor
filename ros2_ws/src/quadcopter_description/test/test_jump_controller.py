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

"""Unit tests for the high-level jumping controller state machine."""

from hopcopter_model.jump_planner import no_slip_ok
import numpy as np
import pytest
from quadcopter_description.jump_controller import (
    JumpController,
    quaternion_from_z_axis,
)

G = 9.80665


def _drop_then_hop(foot_z_high=0.8, q_min=-0.03):
    """Synthesise one drop->stance->takeoff foot/leg sequence."""
    seq = []
    t = 0.0
    dt = 0.005
    # Aerial fall.
    for _ in range(int(0.4 / dt)):
        seq.append((t, foot_z_high, 0.0, 0.0))
        t += dt
    # Landing -> compression -> extension -> takeoff.
    phases = [
        (0.02, 0.0, -3.0),    # LANDING
        (0.02, q_min, -2.0),  # SUPPORT compression
        (0.02, -0.001, 3.0),  # TAKEOFF
        (0.05, -0.001, 1.0),  # TAKEOFF rising
        (0.12, 0.0, 0.0),     # AERIAL
    ]
    for i, (fz, q, qd) in enumerate(phases):
        steps = 4
        for _ in range(steps):
            seq.append((t, fz, q, qd))
            t += dt
    return seq


def _aerial(t0, duration, foot_z=0.5, dt=0.005):
    seq = []
    t = t0
    for _ in range(int(duration / dt)):
        seq.append((t, foot_z, 0.0, 0.0))
        t += dt
    return seq


def _shift(seq, t0):
    return [(t + t0, fz, q, qd) for t, fz, q, qd in seq]


def _feed(controller, seq, pose_z_fn, v_xy=(0.0, 0.0)):
    out = []
    for t, fz, q, qd in seq:
        p = np.array([0.0, 0.0, pose_z_fn(t)])
        out.append(controller.update(t, p, v_xy, fz, q, qd))
    return out


def test_standby_until_first_takeoff():
    ctrl = JumpController(desired_height=0.5, g=G)
    seq = _drop_then_hop()
    z0 = 0.9

    def z_fn(t):
        return max(0.22, z0 - 1.0 * t)

    out = _feed(ctrl, seq, z_fn)
    assert out[0].mode == 'zero_thrust'
    assert np.allclose(out[0].z_b, [0.0, 0.0, 1.0])
    # First cycle has no previous apex height: no powered ascent, so the
    # controller goes straight to the ballistic climb.
    assert any(o.high_state == 'ASCENT_BALLISTIC' for o in out)
    assert out[-1].hop_count == 1


def test_ascent_apex_plan_descent():
    ctrl = JumpController(desired_height=0.5, g=G)
    seq = _drop_then_hop() + _aerial(0.5, 1.6)

    def z_fn(t):
        # Drop then a ballistic hop peaking at ~0.7 m.
        if t < 0.5:
            return max(0.22, 0.9 - 1.0 * t)
        z = 0.22 + 3.0 * (t - 0.5) - 0.5 * G * (t - 0.5) ** 2
        return max(0.22, z)

    out = _feed(ctrl, seq, z_fn)
    states = [o.high_state for o in out]
    modes = [o.mode for o in out]
    assert 'ASCENT_BALLISTIC' in states
    assert 'DESCENT' in states
    assert 'PRE_LAND' in states
    # Ballistic/descent flight keeps attitude stabilised with zero thrust.
    assert 'attitude_only' in modes
    plan = ctrl.last_plan
    assert plan is not None and plan.success
    assert no_slip_ok(plan.z_b_landing, mu=0.8)
    assert no_slip_ok(plan.z_b_takeoff, mu=0.8)


def test_height_control_uses_previous_apex():
    ctrl = JumpController(desired_height=0.7, g=G)
    # Two full cycles: drop, hop, land, drop/hop again.
    seq = (
        _drop_then_hop()
        + _aerial(0.5, 1.1)
        + _shift(_drop_then_hop(), 1.6)
        + _aerial(2.1, 1.6)
    )

    def z_fn(t):
        if t < 0.5:
            return max(0.22, 0.9 - 1.0 * t)
        if t < 1.6:
            u = t - 0.5
        else:
            u = t - 1.6
        z = 0.22 + 3.0 * u - 0.5 * G * u * u
        return max(0.22, z)

    out = _feed(ctrl, seq, z_fn)
    hop_counts = [o.hop_count for o in out]
    assert max(hop_counts) >= 2
    # Second-cycle powered ascent should be commanded (previous apex height
    # is below the 0.7 m target).
    powered_times = [o.t for o in out if o.high_state == 'ASCENT_POWERED'
                     and o.hop_count == 2]
    assert powered_times


def test_quaternion_from_z_axis():
    q = quaternion_from_z_axis(np.array([0.0, 0.0, 1.0]))
    assert np.allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-12)
    z = np.array([0.0, np.sin(0.3), np.cos(0.3)])
    q = quaternion_from_z_axis(z)
    assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-12)
    # Rotating e3 by q should recover z (up to sign conventions: verify
    # with the standard rotation formula).
    w, x, y, zz = q
    r = np.array([
        [1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * w), 2 * (x * zz + y * w)],
        [2 * (x * y + zz * w), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * w)],
        [2 * (x * zz - y * w), 2 * (y * zz + x * w), 1 - 2 * (x * x + y * y)],
    ])
    assert np.allclose(r @ np.array([0.0, 0.0, 1.0]), z, atol=1e-9)


def test_adaptive_tilt_scale_tracks_plant_gain():
    ctrl = JumpController(desired_height=0.6, g=G, tilt_scale=1.0)
    ctrl._update_adaptive_scale(
        np.array([0.5, 0.0, 0.0]), np.array([1.0, 0.0]))
    assert ctrl._tilt_scale_adaptive > 1.0
    ctrl._update_adaptive_scale(
        np.array([0.5, 0.0, 0.0]), np.array([0.2, 0.0]))
    assert ctrl._tilt_scale_adaptive < 1.2


def test_one_hop_correction_is_limited():
    ctrl = JumpController(
        desired_height=0.6, g=G, max_step=0.2,
        ref_fn=lambda t: np.array([2.0, 0.0, 0.0]))
    seq = _drop_then_hop() + _aerial(0.5, 1.6)

    def z_fn(t):
        if t < 0.5:
            return max(0.22, 0.9 - 1.0 * t)
        z = 0.22 + 3.0 * (t - 0.5) - 0.5 * G * (t - 0.5) ** 2
        return max(0.22, z)

    _feed(ctrl, seq, z_fn)
    rows = ctrl.take_rows()
    plan = next(r for r in rows if r['type'] == 'plan')
    dist = np.hypot(float(plan['p_des_x']), float(plan['p_des_y']))
    assert dist <= 0.2 + 1e-6


def test_reference_ramp_soft_starts_first_hops():
    ctrl = JumpController(
        desired_height=0.6, g=G, max_step=10.0, ramp_hops=3,
        ref_fn=lambda t: np.array([2.0, 0.0, 0.0]))
    seq = _drop_then_hop() + _aerial(0.5, 1.6)

    def z_fn(t):
        if t < 0.5:
            return max(0.22, 0.9 - 1.0 * t)
        z = 0.22 + 3.0 * (t - 0.5) - 0.5 * G * (t - 0.5) ** 2
        return max(0.22, z)

    _feed(ctrl, seq, z_fn)
    rows = ctrl.take_rows()
    plan = next(r for r in rows if r['type'] == 'plan')
    # First hop: 1/3 of the 2 m reference displacement.
    assert float(plan['p_des_x']) == pytest.approx(2.0 / 3.0, abs=0.05)
