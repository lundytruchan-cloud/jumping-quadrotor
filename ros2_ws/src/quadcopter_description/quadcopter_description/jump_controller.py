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

"""
High-level jumping controller (paper Eqs. 22-33), pure logic.

The controller runs once per hopping cycle: it waits for the takeoff,
applies the powered ascent ``dt_PA`` (hover thrust keeps the vertical
speed constant), coasts ballistically to the apex, solves for the desired
landing attitude ``z_b(t_LD)`` (numerical optimisation, Eq. 31 area) with
the task-4 stance mapping and the no-slip constraint (Eq. 32), rotates to
that attitude shortly before touchdown, and holds it through the passive
stance until the next takeoff.

The pure class has no ROS dependency so it is unit-testable; the node in
``jump_control_node.py`` provides the message plumbing.
"""

from dataclasses import dataclass
from pathlib import Path
import sys


def _bootstrap_model_path():
    """
    Make the repo-root ``src`` (hopcopter_model) importable.

    The stance model from task 4 lives outside the colcon workspace; this
    bootstrap finds it by walking up from this file to the first directory
    that contains ``src/hopcopter_model``.
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / 'src' / 'hopcopter_model'
        if candidate.is_dir() and (candidate / '__init__.py').is_file():
            src_root = str(parent / 'src')
            if src_root not in sys.path:
                sys.path.insert(0, src_root)
            return


_bootstrap_model_path()

from hopcopter_model.jump_planner import (  # noqa: E402
    ballistic_flight_duration,
    powered_ascent_duration,
    predict_landing_from_apex,
    solve_landing_attitude,
)
from hopcopter_model.params import PAPER_PARAMS  # noqa: E402
import numpy as np  # noqa: E402

from .phase_machine import LegPhase, PhaseStateMachine  # noqa: E402

GRAVITY = 9.80665


@dataclass(frozen=True)
class ControllerOutput:
    """One control decision from :meth:`JumpController.update`."""

    t: float
    mode: str
    z_b: np.ndarray
    high_state: str
    hop_count: int


def quaternion_from_z_axis(z_b, yaw=0.0):
    """Quaternion (w, x, y, z) whose z-axis maps to ``z_b`` (then yaw)."""
    z_b = np.asarray(z_b, dtype=float)
    z = z_b / np.linalg.norm(z_b)
    e3 = np.array([0.0, 0.0, 1.0])
    axis = np.cross(e3, z)
    na = np.linalg.norm(axis)
    if na < 1e-12:
        if z[2] > 0.0:
            q = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            q = np.array([0.0, 1.0, 0.0, 0.0])
    else:
        axis = axis / na
        half = 0.5 * np.arccos(np.clip(z[2], -1.0, 1.0))
        q = np.array([np.cos(half), *(axis * np.sin(half))])
    if yaw:
        half = 0.5 * yaw
        qy = np.array([np.cos(half), 0.0, 0.0, np.sin(half)])
        q = np.array([
            q[0] * qy[0] - q[1] * qy[1] - q[2] * qy[2] - q[3] * qy[3],
            q[0] * qy[1] + q[1] * qy[0] + q[2] * qy[3] - q[3] * qy[2],
            q[0] * qy[2] - q[1] * qy[3] + q[2] * qy[0] + q[3] * qy[1],
            q[0] * qy[3] + q[1] * qy[2] - q[2] * qy[1] + q[3] * qy[0],
        ])
    return q / np.linalg.norm(q)


class JumpController:
    """Per-cycle high-level jumping state machine."""

    def __init__(self, desired_height=0.6, l0=0.22, g=GRAVITY, mu=0.8,
                 pre_landing_lead=0.7, plan_fn=None, ref_fn=None,
                 foot_threshold=0.03, compression_eps=0.002,
                 max_dt_pa=0.8, spin_up_comp=0.05, params=PAPER_PARAMS,
                 max_tilt_deg=18.0, tilt_scale=1.0, fb_gain=0.0,
                 fb_clamp=0.25, fixed_tilt_deg=0.0, max_speed=1.5,
                 max_step=0.25, rotation_time=0.25, ramp_hops=3,
                 adaptive_scale=True):
        self.desired_height = float(desired_height)
        self.l0 = float(l0)
        self.g = float(g)
        self.mu = float(mu)
        self.pre_landing_lead = float(pre_landing_lead)
        self.params = params
        self.max_dt_pa = float(max_dt_pa)
        self.spin_up_comp = float(spin_up_comp)
        self.max_tilt_deg = float(max_tilt_deg)
        self.tilt_scale = float(tilt_scale)
        self.fb_gain = float(fb_gain)
        self.fb_clamp = float(fb_clamp)
        self.fb_decay = 0.7
        self.fixed_tilt_deg = float(fixed_tilt_deg)
        self.max_speed = float(max_speed)
        self.max_step = float(max_step)
        self.rotation_time = float(rotation_time)
        self.ramp_hops = int(ramp_hops)
        self.adaptive_scale = bool(adaptive_scale)

        def default_plan(p_apex, v_apex, p_des, z_d):
            return solve_landing_attitude(
                p_apex, v_apex, p_des, z_d,
                params=self.params, g=self.g, mu=self.mu,
                tilt_limit=np.deg2rad(self.max_tilt_deg))

        self._plan_fn = plan_fn or default_plan
        self._ref_fn = ref_fn or (lambda t: np.array([0.0, 0.0, 0.0]))

        self._machine = PhaseStateMachine(
            foot_threshold=foot_threshold,
            compression_eps=compression_eps,
        )
        self._prev_leg_phase = None
        self._state = 'STANDBY'
        self._hop_count = 0
        self._takeoff_time = None
        self._takeoff_z = None
        self._z_max = None
        self._z_max_t = None
        self._apex_confirmed = False
        self._apex = None          # (t, p_apex, v_xy)
        self._vz_cycle_est = 0.0   # g*(t_apex - t_takeoff) of the last cycle
        self._dt_pa = 0.0
        self._t_powered_end = None
        self._plan = None
        self._t_land_pred = None
        self._tilt_correction = np.zeros(2)
        self._tilt_scale_adaptive = float(self.tilt_scale)
        self._prev_plan = None
        self._landing_ref_k = None
        self._plan_des = None
        self._preland_time = None
        self._pending_rows = []

    @property
    def state(self):
        return self._state

    @property
    def last_plan(self):
        return self._plan

    @property
    def hop_count(self):
        return self._hop_count

    def take_rows(self):
        rows = self._pending_rows
        self._pending_rows = []
        return rows

    def update(self, t, p, v_xy, foot_z, q, q_dot):
        """Advance the controller with one high-rate sample."""
        p = np.asarray(p, dtype=float)
        v_xy = np.asarray(v_xy, dtype=float)
        leg_phase = self._machine.update(t, foot_z, q, q_dot)
        prev = self._prev_leg_phase
        self._prev_leg_phase = leg_phase

        if (prev == LegPhase.TAKEOFF and leg_phase == LegPhase.AERIAL):
            self._on_takeoff(t, p)
        if (prev == LegPhase.AERIAL
                and leg_phase in (LegPhase.LANDING, LegPhase.SUPPORT)):
            self._on_landing(t, p)

        z_b = np.array([0.0, 0.0, 1.0])
        mode = 'zero_thrust'

        if self._state == 'STANDBY':
            pass
        elif self._state == 'ASCENT_POWERED':
            mode = 'hover'
            if self._t_powered_end is not None and t >= self._t_powered_end:
                self._state = 'ASCENT_BALLISTIC'
                mode = 'attitude_only'
        elif self._state == 'ASCENT_BALLISTIC':
            mode = 'attitude_only'
            self._track_apex(t, p)
            if self._apex_confirmed:
                self._on_apex(t, p, v_xy)
                self._state = 'DESCENT'
        elif self._state == 'DESCENT':
            mode = 'attitude_only'
            if (self._t_land_pred is not None
                    and t >= self._t_land_pred - self.pre_landing_lead):
                self._state = 'PRE_LAND'
                self._preland_time = t
                z_b = self._commanded_landing_attitude(t)
            elif (self._t_land_pred is None
                    and leg_phase in (LegPhase.LANDING, LegPhase.SUPPORT)):
                self._state = 'STANCE'
        elif self._state == 'PRE_LAND':
            mode = 'attitude_only'
            z_b = self._commanded_landing_attitude(t)
        elif self._state == 'STANCE':
            z_b = self._commanded_landing_attitude(t)

        return ControllerOutput(
            t=float(t),
            mode=mode,
            z_b=np.array(z_b, dtype=float),
            high_state=self._state,
            hop_count=self._hop_count,
        )

    def _on_takeoff(self, t, p):
        self._hop_count += 1
        self._takeoff_time = t
        self._takeoff_z = float(p[2])
        self._z_max = float(p[2])
        self._z_max_t = t
        self._apex_confirmed = False
        self._apex = None
        # The previous plan targeted the landing that closes this cycle;
        # keep its reference for the feedback correction on that landing.
        self._landing_ref_k = self._plan_des
        self._prev_plan = self._plan
        self._plan = None
        self._plan_des = None
        self._t_land_pred = None

        v_est = self._vz_cycle_est
        base = powered_ascent_duration(
            v_est, self.desired_height, self.g)
        # The simulated motors need ~3 time constants to reach hover speed;
        # add a compensation so the energy burst is not lost in spin-up.
        self._dt_pa = min(
            self.max_dt_pa,
            base + (self.spin_up_comp if base > 0.0 else 0.0),
        )
        self._pending_rows.append({
            'type': 'cycle',
            'cycle': self._hop_count,
            't_takeoff': t,
            'p_takeoff_x': p[0],
            'p_takeoff_y': p[1],
            'p_takeoff_z': p[2],
            'vz_est': v_est,
            'dt_pa': self._dt_pa,
        })
        if self._dt_pa <= 0.0:
            self._state = 'ASCENT_BALLISTIC'
        else:
            self._t_powered_end = t + self._dt_pa
            self._state = 'ASCENT_POWERED'

    def _track_apex(self, t, p):
        z = float(p[2])
        if self._z_max is None or z > self._z_max:
            self._z_max = z
            self._z_max_t = t
            self._apex_confirmed = False
        elif (not self._apex_confirmed
                and self._z_max - (self._takeoff_z or 0.0) > 0.03
                and t - self._z_max_t > 0.05
                and z < self._z_max - 0.005):
            self._apex_confirmed = True

    def _on_apex(self, t, p, v_xy):
        # Effective takeoff speed that produces the measured apex height
        # ballistically.  This is more reliable than the flight-time-based
        # estimate (the leg/body motion at takeoff biases the timing) and
        # matches the energy-balance form of Eqs. 27-29.
        h_meas = max(0.0, float(p[2]) - (self._takeoff_z or 0.0))
        v_est = np.sqrt(2.0 * self.g * h_meas)
        self._vz_cycle_est = max(0.0, v_est)
        self._apex = (t, np.array(p, dtype=float), np.array(v_xy, dtype=float))

        dt_pa_base = powered_ascent_duration(
            self._vz_cycle_est, self.desired_height, self.g)
        dt_pa_next = dt_pa_base + (
            self.spin_up_comp if dt_pa_base > 0.0 else 0.0)
        dt_pj_next = ballistic_flight_duration(
            self._vz_cycle_est, self.desired_height, self.g)
        # Reference for the next landing is evaluated at the expected next
        # landing time ("take the next-cycle expected position").  The fall
        # time is estimated with an upright landing attitude; the small
        # attitude-dependent variation (<= ~40 ms) is negligible.
        e3 = np.array([0.0, 0.0, 1.0])
        try:
            t_fall_est = predict_landing_from_apex(
                np.array(p, dtype=float),
                np.array([v_xy[0], v_xy[1], 0.0], dtype=float),
                e3, self.l0, self.g).t_fall
        except ValueError:
            t_fall_est = 0.0
        # Paper Eq. 33: the next landing time is the current time-to-landing
        # plus one full ballistic cycle at the setpoint altitude.
        t_land_next = (
            t + t_fall_est
            + 2.0 * np.sqrt(2.0 * self.desired_height / self.g))
        p_des = np.asarray(self._ref_fn(t_land_next), dtype=float)
        # Soft start: scale the first reference displacements so the robot
        # joins the trajectory without building up a large horizontal speed
        # on the very first hops.
        if self.ramp_hops > 0:
            s = min(1.0, self._hop_count / float(self.ramp_hops))
            p_des[:2] = p[:2] + s * (p_des[:2] - p[:2])
        # Speed governor: if the horizontal speed already exceeds the track
        # speed, pull the target inward so the robot decelerates instead of
        # compounding the plant gain.
        v_h = np.hypot(v_xy[0], v_xy[1])
        if v_h > self.max_speed:
            s = self.max_speed / v_h
            p_des[:2] = p[:2] + s * (p_des[:2] - p[:2])
        # One-hop correction limit: never ask a single stance to travel more
        # than ``max_step`` (the paper introduces intermediate setpoints when
        # the target is unreachable in one hop).
        disp = p_des[:2] - p[:2]
        dist = np.hypot(disp[0], disp[1])
        if dist > self.max_step:
            p_des[:2] = p[:2] + disp * (self.max_step / dist)
        try:
            plan = self._plan_fn(
                np.array(p, dtype=float),
                np.array([v_xy[0], v_xy[1], 0.0], dtype=float),
                p_des,
                self.desired_height,
            )
        except Exception as exc:  # planner must never kill the controller
            self._pending_rows.append({
                'type': 'plan_error',
                'cycle': self._hop_count,
                't_apex': t,
                'error': str(exc),
            })
            plan = None

        if plan is not None and plan.success:
            self._plan = plan
            self._t_land_pred = t + plan.t_fall_current
            self._pending_rows.append({
                'type': 'plan',
                'cycle': self._hop_count,
                't_apex': t,
                'p_apex_x': p[0],
                'p_apex_y': p[1],
                'z_apex': p[2],
                'z_d': self.desired_height,
                't_land_pred': self._t_land_pred,
                'p_des_x': p_des[0],
                'p_des_y': p_des[1],
                'z_b_ld_x': plan.z_b_landing[0],
                'z_b_ld_y': plan.z_b_landing[1],
                'z_b_ld_z': plan.z_b_landing[2],
                'z_b_to_x': plan.z_b_takeoff[0],
                'z_b_to_y': plan.z_b_takeoff[1],
                'z_b_to_z': plan.z_b_takeoff[2],
                'p_land_next_x': plan.p_landing_next_pred[0],
                'p_land_next_y': plan.p_landing_next_pred[1],
                'dt_pa_next': dt_pa_next,
                'dt_pj_next': dt_pj_next,
                'tilt_scale': self._tilt_scale_adaptive,
            })
            self._plan_des = p_des
        else:
            self._plan = None
            self._plan_des = None
            self._t_land_pred = None

    def _on_landing(self, t, p):
        if self._state in ('PRE_LAND', 'DESCENT'):
            self._state = 'STANCE'
        err_xy = float('nan')
        err_ref = float('nan')
        if self._plan is not None:
            err_xy = np.hypot(
                p[0] - self._plan.p_landing_k[0],
                p[1] - self._plan.p_landing_k[1],
            )
        if self._landing_ref_k is not None:
            err_ref = np.hypot(
                p[0] - self._landing_ref_k[0],
                p[1] - self._landing_ref_k[1],
            )
            e = np.array([
                self._landing_ref_k[0] - p[0],
                self._landing_ref_k[1] - p[1],
            ])
            self._tilt_correction = np.clip(
                self.fb_decay * self._tilt_correction
                + self.fb_gain * e,
                -self.fb_clamp, self.fb_clamp)
            self._landing_ref_k = None
        # Online gain calibration: compare the displacement predicted by the
        # previous plan with the displacement actually achieved.
        if self._prev_plan is not None and self.adaptive_scale:
            model_disp = (
                self._prev_plan.p_landing_next_pred
                - self._prev_plan.p_landing_k)
            actual_disp = p[:2] - self._prev_plan.p_landing_k[:2]
            self._update_adaptive_scale(model_disp, actual_disp)
            self._prev_plan = None
        self._pending_rows.append({
            'type': 'landing',
            'cycle': self._hop_count,
            't_landing': t,
            'p_land_x': p[0],
            'p_land_y': p[1],
            'p_land_z': p[2],
            'err_xy_pred': err_xy,
            'err_ref': err_ref,
            'tilt_corr_x': self._tilt_correction[0],
            'tilt_corr_y': self._tilt_correction[1],
            'tilt_scale': self._tilt_scale_adaptive,
        })

    def _commanded_landing_attitude(self, t=None):
        if self.fixed_tilt_deg > 0.0:
            a = np.deg2rad(self.fixed_tilt_deg)
            return np.array([np.sin(a), 0.0, np.cos(a)])
        if self._plan is None:
            return np.array([0.0, 0.0, 1.0])
        tilt = (self._plan.z_b_landing - np.array([0.0, 0.0, 1.0]))
        tilt *= self._tilt_scale_adaptive
        tilt[:2] += self._tilt_correction
        z = np.array([0.0, 0.0, 1.0]) + tilt
        # Never command a landing attitude beyond the tuning cap, even with
        # the feedback term (avoids tumbling for saturated corrections).
        alpha = float(np.arccos(np.clip(z[2] / np.linalg.norm(z), -1.0, 1.0)))
        if alpha > np.deg2rad(self.max_tilt_deg):
            h = np.hypot(z[0], z[1])
            if h > 1e-12:
                s = np.tan(np.deg2rad(self.max_tilt_deg)) / h
                z = np.array([s * z[0], s * z[1], 1.0])
        z = z / np.linalg.norm(z)
        # Ramp the setpoint from upright to the target over ``rotation_time``
        # so the underdamped ballistic attitude loop is not step-excited.
        if t is not None and self._preland_time is not None:
            s = float(np.clip(
                (t - self._preland_time) / self.rotation_time, 0.0, 1.0))
            z = np.array([0.0, 0.0, 1.0]) + s * (z - np.array([0.0, 0.0, 1.0]))
        return z / np.linalg.norm(z)

    def _update_adaptive_scale(self, model_disp, actual_disp):
        """Leaky gain calibration from achieved vs model displacement."""
        model_norm = float(np.linalg.norm(model_disp[:2]))
        actual_norm = float(np.linalg.norm(actual_disp[:2]))
        if model_norm < 0.02:
            return
        ratio = actual_norm / model_norm
        self._tilt_scale_adaptive = float(np.clip(
            self._tilt_scale_adaptive * (0.7 + 0.3 * ratio),
            0.2, 2.0))
