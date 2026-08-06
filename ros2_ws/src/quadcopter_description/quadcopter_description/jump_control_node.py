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

"""High-level jumping controller node (paper Eqs. 22-33)."""

from collections import deque
import csv
import threading

from geometry_msgs.msg import PoseStamped, QuaternionStamped
import numpy as np
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Float32Array
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from .jump_controller import JumpController, quaternion_from_z_axis

CSV_COLUMNS = [
    'type', 'cycle',
    't_takeoff', 'p_takeoff_x', 'p_takeoff_y', 'p_takeoff_z',
    'vz_est', 'dt_pa',
    't_apex', 'p_apex_x', 'p_apex_y', 'z_apex', 'z_d', 't_land_pred',
    'p_des_x', 'p_des_y',
    'z_b_ld_x', 'z_b_ld_y', 'z_b_ld_z',
    'z_b_to_x', 'z_b_to_y', 'z_b_to_z',
    'p_land_next_x', 'p_land_next_y',
    'dt_pa_next', 'dt_pj_next',
    't_landing', 'p_land_x', 'p_land_y', 'p_land_z', 'err_xy_pred',
    'err_ref', 'tilt_corr_x', 'tilt_corr_y',
    'tilt_scale',
    'error',
]


def _parse_targets(s):
    targets = []
    for pair in s.split(';'):
        pair = pair.strip()
        if not pair:
            continue
        x, y = pair.split(',')
        targets.append((float(x), float(y)))
    return targets


class JumpControlNode(Node):
    """Drive the low-level attitude/thrust modes through hopping cycles."""

    def __init__(self):
        super().__init__('jump_control_node')
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('pose_topic', '/quadcopter/pose')
        self.declare_parameter('leg_state_topic', '/quadcopter/leg_state')
        self.declare_parameter('setpoint_topic', '/quadcopter/attitude_setpoint')
        self.declare_parameter('mode_topic', '/quadcopter/control_mode')
        self.declare_parameter('done_topic', '/quadcopter/jump_done')
        self.declare_parameter('desired_height', 0.6)
        self.declare_parameter('trajectory', 'spot')
        self.declare_parameter('circle_radius', 0.5)
        self.declare_parameter('circle_omega', 0.9)
        self.declare_parameter('circle_phase0', 0.0)
        self.declare_parameter('step_targets', '0,0;0.6,0')
        self.declare_parameter('step_hold', 2.0)
        self.declare_parameter('num_hops', 0)
        self.declare_parameter('duration', 0.0)
        self.declare_parameter('pre_landing_lead', 0.4)
        self.declare_parameter('mu', 0.8)
        self.declare_parameter('spin_up_comp', 0.05)
        self.declare_parameter('max_tilt_deg', 18.0)
        self.declare_parameter('tilt_scale', 1.0)
        self.declare_parameter('fb_gain', 0.0)
        self.declare_parameter('fb_clamp', 0.25)
        self.declare_parameter('fixed_tilt_deg', 0.0)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_step', 0.25)
        self.declare_parameter('rotation_time', 0.25)
        self.declare_parameter('ramp_hops', 3)
        self.declare_parameter('l0', 0.22)
        self.declare_parameter('g', 9.80665)
        self.declare_parameter('log_file', '')

        self._rate = self.get_parameter('control_rate_hz').value
        self._num_hops = int(self.get_parameter('num_hops').value)
        self._duration = self.get_parameter('duration').value
        self._log_file = self.get_parameter('log_file').value
        self._sim_time = 0.0
        self._start_time = None
        self._pose = None
        self._pose_history = deque(maxlen=60)
        self._leg = None
        self._finished = False
        self._csv_file = None
        self._csv_writer = None

        trajectory = self.get_parameter('trajectory').value
        radius = self.get_parameter('circle_radius').value
        omega = self.get_parameter('circle_omega').value
        phase0 = self.get_parameter('circle_phase0').value
        targets = _parse_targets(self.get_parameter('step_targets').value)
        hold = self.get_parameter('step_hold').value

        # Imported here: ``jump_controller`` already bootstrapped the
        # repo-root ``src`` path for the task-4 model package.
        from hopcopter_model.jump_planner import (
            circle_reference,
            step_reference,
        )

        if trajectory == 'circle':
            def ref_fn(t):
                return circle_reference(
                    [t], radius=radius, omega=omega, phase0=phase0)[0]
        elif trajectory == 'step':
            def ref_fn(t):
                return step_reference([t], targets, hold_period=hold)[0]
        else:
            def ref_fn(t):
                return np.array([0.0, 0.0, 0.0])

        self._controller = JumpController(
            desired_height=self.get_parameter('desired_height').value,
            l0=self.get_parameter('l0').value,
            g=self.get_parameter('g').value,
            mu=self.get_parameter('mu').value,
            pre_landing_lead=self.get_parameter('pre_landing_lead').value,
            spin_up_comp=self.get_parameter('spin_up_comp').value,
            max_tilt_deg=self.get_parameter('max_tilt_deg').value,
            tilt_scale=self.get_parameter('tilt_scale').value,
            fb_gain=self.get_parameter('fb_gain').value,
            fb_clamp=self.get_parameter('fb_clamp').value,
            fixed_tilt_deg=self.get_parameter('fixed_tilt_deg').value,
            max_speed=self.get_parameter('max_speed').value,
            max_step=self.get_parameter('max_step').value,
            rotation_time=self.get_parameter('rotation_time').value,
            ramp_hops=self.get_parameter('ramp_hops').value,
            ref_fn=ref_fn,
        )

        self._setpoint_pub = self.create_publisher(
            QuaternionStamped, self.get_parameter('setpoint_topic').value, 10)
        self._mode_pub = self.create_publisher(
            String, self.get_parameter('mode_topic').value, 10)
        self._done_pub = self.create_publisher(
            String, self.get_parameter('done_topic').value, 1)

        self._pose_sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self._on_pose, 10)
        self._leg_sub = self.create_subscription(
            Float32Array, self.get_parameter('leg_state_topic').value,
            self._on_leg, 100)
        self._clock_sub = self.create_subscription(
            Clock, '/clock', self._on_clock, 10)

        if self._log_file:
            self._csv_file = open(self._log_file, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(CSV_COLUMNS)

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            'Jump controller started: trajectory={}, z_d={:.2f} m, '
            'num_hops={}, log={}'.format(
                trajectory, self._controller.desired_height,
                self._num_hops, self._log_file or '(none)'))

    def _on_clock(self, msg):
        self._sim_time = msg.clock.sec + 1e-9 * msg.clock.nanosec
        if self._start_time is None:
            self._start_time = self._sim_time

    def _on_pose(self, msg):
        self._pose = msg.pose
        self._pose_history.append((
            self._sim_time,
            msg.pose.position.x,
            msg.pose.position.y,
        ))

    def _on_leg(self, msg):
        if len(msg.data) >= 5:
            self._leg = msg.data

    def _velocity_estimate(self):
        if len(self._pose_history) < 2:
            return np.zeros(2)
        newest = self._pose_history[-1]
        oldest = None
        for item in self._pose_history:
            if newest[0] - item[0] >= 0.08:
                oldest = item
        if oldest is None:
            oldest = self._pose_history[0]
        dt = newest[0] - oldest[0]
        if dt < 0.02 or dt > 1.0:
            return np.zeros(2)
        return np.array([
            (newest[1] - oldest[1]) / dt,
            (newest[2] - oldest[2]) / dt,
        ])

    def _publish(self, z_b, mode):
        q = quaternion_from_z_axis(z_b, yaw=0.0)
        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.quaternion.w = float(q[0])
        msg.quaternion.x = float(q[1])
        msg.quaternion.y = float(q[2])
        msg.quaternion.z = float(q[3])
        self._setpoint_pub.publish(msg)
        mode_msg = String()
        mode_msg.data = mode
        self._mode_pub.publish(mode_msg)

    def _tick(self):
        if self._finished:
            return
        if self._pose is None or self._leg is None:
            self._publish(np.array([0.0, 0.0, 1.0]), 'zero_thrust')
            return

        p = np.array([
            self._pose.position.x,
            self._pose.position.y,
            self._pose.position.z,
        ])
        v_xy = self._velocity_estimate()
        foot_z = float(self._leg[4])
        q = float(self._leg[1])
        q_dot = float(self._leg[2])
        out = self._controller.update(
            self._sim_time, p, v_xy, foot_z, q, q_dot)
        self._publish(out.z_b, out.mode)

        for row in self._controller.take_rows():
            self._log_row(row)

        if self._duration > 0.0 and self._start_time is not None:
            if self._sim_time - self._start_time >= self._duration:
                self._finish('duration')
                return
        if (self._num_hops > 0
                and self._controller.hop_count >= self._num_hops
                and self._controller.state in ('STANCE', 'STANDBY')):
            self._finish('hops')

    def _log_row(self, row):
        if self._csv_writer is None:
            return
        self._csv_writer.writerow([
            row.get(k, '') for k in CSV_COLUMNS])
        self._csv_file.flush()

    def _finish(self, reason):
        self._finished = True
        self._publish(np.array([0.0, 0.0, 1.0]), 'zero_thrust')
        done = String()
        done.data = (
            'reason={} hops={} t={:.3f}'.format(
                reason, self._controller.hop_count, self._sim_time))
        self._done_pub.publish(done)
        self.get_logger().info(
            'Jump controller finished: {} at t={:.3f}s, hops={}'.format(
                reason, self._sim_time, self._controller.hop_count))
        self._close_log()
        self.destroy_timer(self._timer)
        threading.Thread(target=rclpy.shutdown, daemon=True).start()

    def _close_log(self):
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_writer = None


def main(args=None):
    rclpy.init(args=args)
    node = JumpControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
