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

"""Task-8 tests: stabilizer activation, strategy modes, no-position mode."""

import numpy as np
import pytest

from quadcopter_description.jump_controller import JumpController

G = 9.80665


def _drop_then_hop(foot_z_high=0.8, q_min=-0.03):
    seq = []
    t = 0.0
    dt = 0.005
    for _ in range(int(0.4 / dt)):
        seq.append((t, foot_z_high, 0.0, 0.0))
        t += dt
    phases = [
        (0.02, 0.0, -3.0),
        (0.02, q_min, -2.0),
        (0.02, -0.001, 3.0),
        (0.05, -0.001, 1.0),
        (0.12, 0.0, 0.0),
    ]
    for fz, q, qd in phases:
        for _ in range(4):
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


def _aerial_landing(t0, duration, dt=0.005):
    """Aerial phase that drops the foot to the ground at the end."""
    seq = []
    n = int(duration / dt)
    for i in range(n):
        t = t0 + i * dt
        foot = max(0.0, 0.5 - 0.5 * i / n)
        seq.append((t, foot, 0.0, 0.0))
    return seq


def _shift(seq, t0):
    return [(t + t0, fz, q, qd) for t, fz, q, qd in seq]


def _feed(controller, seq, z_fn, v_xy=(0.0, 0.0)):
    out = []
    for t, fz, q, qd in seq:
        p = np.array([0.0, 0.0, z_fn(t)])
        out.append(controller.update(t, p, v_xy, fz, q, qd))
    return out


def _two_cycles(ctrl):
    seq = (
        _drop_then_hop()
        + _aerial_landing(0.5, 1.0)
        + _shift(_drop_then_hop(), 1.5)
        + _aerial_landing(2.0, 1.2)
    )

    def z_fn(t):
        if t < 0.5:
            return max(0.22, 0.9 - 1.0 * t)
        if t < 1.5:
            u = t - 0.5
        else:
            u = t - 1.5
        return max(0.22, 0.22 + 3.0 * u - 0.5 * G * u * u)

    return _feed(ctrl, seq, z_fn)


def test_combined_activates_stabilizer_only_during_descent():
    ctrl = JumpController(desired_height=0.5, g=G, control_strategy='combined')
    out = _two_cycles(ctrl)
    states = [o.high_state for o in out]
    for o, state in zip(out, states):
        expected = state in ('DESCENT', 'PRE_LAND', 'STANCE')
        assert o.stabilizer_active is expected
    assert 'DESCENT' in states
    assert 'ASCENT_POWERED' in states or 'ASCENT_BALLISTIC' in states


def test_attitude_only_never_activates_stabilizer():
    ctrl = JumpController(
        desired_height=0.5, g=G, control_strategy='attitude_only')
    out = _two_cycles(ctrl)
    assert all(not o.stabilizer_active for o in out)
    # Attitude loop stays on during the descent.
    descent = [o for o in out if o.high_state == 'DESCENT']
    assert descent and all(o.mode == 'attitude_only' for o in descent)


def test_stabilizer_only_switches_attitude_off_during_descent():
    ctrl = JumpController(
        desired_height=0.5, g=G, control_strategy='stabilizer_only')
    out = _two_cycles(ctrl)
    assert all(o.stabilizer_active for o in out)
    descent = [o for o in out if o.high_state == 'DESCENT']
    assert descent and all(o.mode == 'zero_thrust' for o in descent)
    powered = [o for o in out if o.high_state == 'ASCENT_POWERED']
    if powered:
        assert all(o.mode == 'hover' for o in powered)


def test_no_position_feedback_open_loop_cycle():
    ctrl = JumpController(
        desired_height=0.5, g=G, control_strategy='combined',
        position_feedback=False, no_fb_dt_pa=0.15)
    out = _two_cycles(ctrl)
    rows = ctrl.take_rows()
    # No planning happens without position feedback.
    assert not any(r['type'] == 'plan' for r in rows)
    assert ctrl.hop_count >= 2
    # Setpoint stays upright and the stabilizer activates on the descent.
    for o in out:
        assert np.allclose(o.z_b, [0.0, 0.0, 1.0], atol=1e-12)
    descent = [o for o in out if o.high_state == 'DESCENT']
    assert descent and all(o.stabilizer_active for o in descent)
    powered = [o for o in out if o.high_state == 'ASCENT_POWERED']
    assert powered


def test_no_position_feedback_ignores_pose_and_velocity():
    ctrl = JumpController(
        desired_height=0.5, g=G, control_strategy='combined',
        position_feedback=False, no_fb_dt_pa=0.15)
    seq = _drop_then_hop() + _aerial_landing(0.5, 1.0)

    def z_fn(t):
        return 100.0 + t  # absurd pose must not affect the controller

    out = _feed(ctrl, seq, z_fn, v_xy=(99.0, -99.0))
    assert ctrl.last_plan is None
    assert ctrl.hop_count >= 1
    assert all(np.allclose(o.z_b, [0.0, 0.0, 1.0], atol=1e-12) for o in out)


def test_no_feedback_adapts_descent_timing_to_measured_cycle():
    ctrl = JumpController(
        desired_height=0.5, g=G, control_strategy='combined',
        position_feedback=False, no_fb_dt_pa=None)
    # First cycle: takeoff near t=0.44, aerial that lands at ~1.24 s.
    seq = _drop_then_hop() + _aerial_landing(0.5, 0.8)
    _feed(ctrl, seq, lambda t: 0.5)
    assert ctrl.hop_count >= 1
    assert ctrl._vz_no_fb_est > 0.0
    # Second takeoff uses the adapted apex estimate.
    seq2 = seq + _shift(_drop_then_hop(), 1.5) + _aerial_landing(2.0, 0.8)
    _feed(ctrl, seq2, lambda t: 0.5)
    assert ctrl.hop_count >= 2
    assert 0.0 < ctrl._dt_pa <= ctrl.max_dt_pa


def test_no_feedback_uses_takeoff_leg_speed_for_vz():
    ctrl = JumpController(
        desired_height=0.5, g=G, control_strategy='combined',
        position_feedback=False, no_fb_dt_pa=None)
    ctrl._on_takeoff(1.0, np.zeros(3), q_dot=3.0)
    assert ctrl._vz_no_fb_est == pytest.approx(1.5, rel=1e-9)
    ctrl._on_takeoff(2.0, np.zeros(3), q_dot=3.0)
    assert ctrl._vz_no_fb_est == pytest.approx(2.25, rel=1e-9)
    # A slow extension is treated as noise and ignored.
    ctrl._on_takeoff(3.0, np.zeros(3), q_dot=0.1)
    assert ctrl._vz_no_fb_est == pytest.approx(2.25, rel=1e-9)
