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
Low-level attitude control node (hover / zero-thrust modes).

IMU attitude and rate -> attitude PID -> attitude-priority thrust allocation
(paper Eqs. 34-35) -> per-motor speed commands on the Gazebo actuator topic.
"""

import csv

from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, QuaternionStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String

from .attitude_controller import AttitudePidController
from .attitude_math import quaternion_normalize, quaternion_to_rpy
from .thrust_allocation import ThrustAllocation

GRAVITY = 9.80665


class AttitudeControlNode(Node):
    """Publish motor speed commands from IMU attitude and a setpoint."""

    def __init__(self):
        super().__init__('attitude_control_node')
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('imu_topic', '/quadcopter/imu')
        self.declare_parameter('setpoint_topic', '/quadcopter/attitude_setpoint')
        self.declare_parameter('mode_topic', '/quadcopter/control_mode')
        self.declare_parameter('motor_cmd_topic', '/quadcopter/motor_speed')
        self.declare_parameter('pose_topic', '/quadcopter/pose')
        self.declare_parameter('mass', 0.0348)
        self.declare_parameter('motor_constant', 3.0e-8)
        self.declare_parameter('max_rot_velocity', 2200.0)
        self.declare_parameter('arm', 0.032527)
        self.declare_parameter('moment_ratio', 0.006)
        self.declare_parameter('kp_att', 15.0)
        self.declare_parameter('kp_rate', 1.2e-3)
        self.declare_parameter('ki_rate', 1.0e-3)
        self.declare_parameter('kd_rate', 0.0)
        self.declare_parameter('integral_limit', 1.0e-2)
        self.declare_parameter('max_torque', 0.05)
        self.declare_parameter('att_only_torque_scale', 1.0)
        self.declare_parameter('att_only_max_thrust', 0.25)
        self.declare_parameter('att_only_yaw_gain', 0.0)
        self.declare_parameter('att_only_thrust_scale', 0.1)
        self.declare_parameter('imu_timeout', 0.2)
        self.declare_parameter('altitude_hold', True)
        self.declare_parameter('alt_kp', 0.6)
        self.declare_parameter('alt_kd', 0.3)
        self.declare_parameter('alt_target', 0.0)
        self.declare_parameter('alt_min_scale', 0.5)
        self.declare_parameter('alt_max_scale', 1.7)
        self.declare_parameter('log_file', '')

        self._rate = self.get_parameter('control_rate_hz').value
        self._mass = self.get_parameter('mass').value
        self._motor_constant = self.get_parameter('motor_constant').value
        self._max_speed = self.get_parameter('max_rot_velocity').value
        self._att_only_torque_scale = self.get_parameter(
            'att_only_torque_scale').value
        self._att_only_max_thrust = self.get_parameter(
            'att_only_max_thrust').value
        self._att_only_yaw_gain = self.get_parameter(
            'att_only_yaw_gain').value
        self._att_only_thrust_scale = self.get_parameter(
            'att_only_thrust_scale').value
        self._imu_timeout = self.get_parameter('imu_timeout').value
        self._altitude_hold = self.get_parameter('altitude_hold').value
        self._alt_kp = self.get_parameter('alt_kp').value
        self._alt_kd = self.get_parameter('alt_kd').value
        self._alt_target = self.get_parameter('alt_target').value
        self._alt_min_scale = self.get_parameter('alt_min_scale').value
        self._alt_max_scale = self.get_parameter('alt_max_scale').value
        self._log_file = self.get_parameter('log_file').value

        self._pid = AttitudePidController(
            kp_att=self.get_parameter('kp_att').value,
            kp_rate=self.get_parameter('kp_rate').value,
            ki_rate=self.get_parameter('ki_rate').value,
            kd_rate=self.get_parameter('kd_rate').value,
            integral_limit=self.get_parameter('integral_limit').value,
            max_torque=self.get_parameter('max_torque').value,
        )
        self._allocator = ThrustAllocation(
            arm=self.get_parameter('arm').value,
            moment_ratio=self.get_parameter('moment_ratio').value,
        )

        self._sim_time = 0.0
        self._last_control_time = None
        self._q_cur = None
        self._omega = None
        self._last_imu_time = None
        self._q_ref = (1.0, 0.0, 0.0, 0.0)
        self._mode = 'hover'
        self._pose = None
        self._z_target = self._alt_target if self._alt_target > 0.0 else None
        self._prev_z = None
        self._prev_z_time = None
        self._vz = 0.0
        self._stale_warned = False
        self._log_counter = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self._imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._on_imu, sensor_qos)
        self._setpoint_sub = self.create_subscription(
            QuaternionStamped,
            self.get_parameter('setpoint_topic').value,
            self._on_setpoint, 10)
        self._mode_sub = self.create_subscription(
            String, self.get_parameter('mode_topic').value,
            self._on_mode, 10)
        self._pose_sub = self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value,
            self._on_pose, 10)
        self._clock_sub = self.create_subscription(
            Clock, '/clock', self._on_clock, 10)

        self._motor_pub = self.create_publisher(
            Actuators, self.get_parameter('motor_cmd_topic').value, 10)
        self._diag_pub = self.create_publisher(
            Float32MultiArray, '/quadcopter/control_state', 10)

        self._csv_file = None
        self._csv_writer = None
        if self._log_file:
            self._csv_file = open(self._log_file, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'sim_time', 'mode',
                'qx', 'qy', 'qz', 'qw',
                'r_ref_x', 'r_ref_y', 'r_ref_z', 'r_ref_w',
                'roll', 'pitch', 'yaw',
                'erot_x', 'erot_y', 'erot_z',
                'omega_x', 'omega_y', 'omega_z',
                'tau_x', 'tau_y', 'tau_z',
                'thrust_des', 'thrust_actual',
                'f0', 'f1', 'f2', 'f3',
                'speed0', 'speed1', 'speed2', 'speed3',
                'vz', 'z_target', 'pos_x', 'pos_y', 'pos_z',
            ])

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            'Attitude control node started: rate={:.1f} Hz, mode={}, '
            'log={}'.format(self._rate, self._mode, self._log_file or '(none)'),
        )

    def _on_clock(self, msg):
        self._sim_time = msg.clock.sec + 1e-9 * msg.clock.nanosec

    def _on_imu(self, msg):
        q = msg.orientation
        self._q_cur = quaternion_normalize(
            (q.w, q.x, q.y, q.z))
        self._omega = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        self._last_imu_time = self._sim_time

    def _on_setpoint(self, msg):
        q = msg.quaternion
        self._q_ref = quaternion_normalize((q.w, q.x, q.y, q.z))

    def _on_mode(self, msg):
        mode = msg.data.strip().lower()
        if mode in ('hover', 'zero_thrust', 'attitude_only'):
            if mode != self._mode:
                self.get_logger().info(
                    'control mode -> {} at t={:.3f}s'.format(
                        mode, self._sim_time))
            self._mode = mode
        else:
            self.get_logger().error(
                'unknown control mode: {} (keep {})'.format(
                    msg.data, self._mode))

    def _on_pose(self, msg):
        self._pose = msg.pose
        z = msg.pose.position.z
        if self._prev_z is not None and self._prev_z_time is not None:
            dt = self._sim_time - self._prev_z_time
            if 0.001 < dt < 1.0:
                vz_raw = (z - self._prev_z) / dt
                self._vz = 0.8 * self._vz + 0.2 * vz_raw
        self._prev_z = z
        self._prev_z_time = self._sim_time
        if self._z_target is None:
            self._z_target = z
            self.get_logger().info(
                'altitude target set to {:.3f} m'.format(z))

    def _tick(self):
        now = self._sim_time
        if self._last_control_time is None:
            dt = 1.0 / self._rate
        else:
            dt = now - self._last_control_time
        dt = max(0.001, min(dt, 0.1))
        self._last_control_time = now

        if (self._q_cur is None or self._omega is None
                or self._last_imu_time is None
                or now - self._last_imu_time > self._imu_timeout):
            if not self._stale_warned:
                self.get_logger().warn(
                    'IMU data missing/stale, commanding zero motors')
                self._stale_warned = True
            self._publish_speeds([0.0, 0.0, 0.0, 0.0])
            return
        self._stale_warned = False

        if self._mode == 'zero_thrust':
            torque = (0.0, 0.0, 0.0)
            thrust_desired = 0.0
        elif self._mode == 'attitude_only':
            # Ballistic segment: keep the attitude stabilised while the
            # net thrust stays small (paper: "thrust of only ~mg/10 was
            # required to obtain satisfactory attitude control performance").
            self._pid.yaw_gain = self._att_only_yaw_gain
            self._pid.yaw_rate_gain = self._att_only_yaw_gain
            torque = self._pid.update(
                self._q_ref, self._q_cur, self._omega, dt)
            self._pid.yaw_gain = 1.0
            self._pid.yaw_rate_gain = 1.0
            # With no base thrust the attitude-priority allocator would push
            # motors to their limits for large errors and produce a net
            # thrust (a "rocket" effect).  Scale the torque demand so it
            # stays within the achievable differential range and cap the
            # resulting net thrust afterwards.
            torque = tuple(
                self._att_only_torque_scale * t for t in torque)
            thrust_desired = (
                self._att_only_thrust_scale * self._mass * GRAVITY)
        else:
            thrust_desired = self._mass * GRAVITY
            if self._altitude_hold and self._z_target is not None:
                z = (self._pose.position.z
                     if self._pose is not None else self._z_target)
                base = self._mass * GRAVITY
                thrust_desired = (
                    base
                    + self._alt_kp * (self._z_target - z)
                    - self._alt_kd * self._vz)
                thrust_desired = max(
                    base * self._alt_min_scale,
                    min(base * self._alt_max_scale, thrust_desired))
            torque = self._pid.update(
                self._q_ref, self._q_cur, self._omega, dt)

        result = self._allocator.allocate(thrust_desired, torque)
        if (self._mode == 'attitude_only'
                and result['thrust'] > self._att_only_max_thrust):
            scale = self._att_only_max_thrust / result['thrust']
            result = {
                **result,
                'f': [scale * f for f in result['f']],
                'thrust': scale * result['thrust'],
            }
        speeds = self._allocator.speeds_from_thrust(
            result['f'], self._motor_constant, self._max_speed)
        self._publish_speeds(speeds)
        self._publish_diagnostics(torque, thrust_desired, result)
        self._log(result, torque, thrust_desired, speeds)

    def _publish_speeds(self, speeds):
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.velocity = [float(s) for s in speeds]
        self._motor_pub.publish(msg)

    def _publish_diagnostics(self, torque, thrust_desired, result):
        msg = Float32MultiArray()
        error = self._pid.last_attitude_error
        msg.data = [
            float(error[0]), float(error[1]), float(error[2]),
            float(torque[0]), float(torque[1]), float(torque[2]),
            float(thrust_desired), float(result['thrust']),
            {
                'hover': 1.0,
                'zero_thrust': 0.0,
                'attitude_only': 2.0,
            }.get(self._mode, 0.0),
        ]
        self._diag_pub.publish(msg)

    def _log(self, result, torque, thrust_desired, speeds):
        if self._csv_writer is None:
            return
        roll, pitch, yaw = quaternion_to_rpy(self._q_cur)
        error = self._pid.last_attitude_error
        qr = self._q_ref
        q = self._q_cur
        row = [self._sim_time, self._mode]
        row.extend([q[1], q[2], q[3], q[0]])
        row.extend([qr[1], qr[2], qr[3], qr[0]])
        row.extend([roll, pitch, yaw])
        row.extend(error)
        row.extend(self._omega)
        row.extend(torque)
        row.extend([thrust_desired, result['thrust']])
        row.extend(result['f'])
        row.extend(speeds)
        row.extend([self._vz, self._z_target if self._z_target is not None
                    else float('nan')])
        if self._pose is not None:
            row.extend([
                self._pose.position.x,
                self._pose.position.y,
                self._pose.position.z,
            ])
        else:
            row.extend([float('nan')] * 3)
        self._csv_writer.writerow(row)
        self._log_counter += 1
        if self._log_counter % 500 == 0:
            self._csv_file.flush()

    def stop_motors(self):
        """Failsafe: command all motors to zero."""
        try:
            self._publish_speeds([0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass

    def close_log(self):
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_writer = None


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
