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

"""Verification driver: publishes attitude/mode steps on a fixed schedule."""

import threading

from geometry_msgs.msg import QuaternionStamped
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from .attitude_math import quaternion_from_rpy


class AttitudeSetpointNode(Node):
    """Step schedule: hover -> roll -> pitch -> yaw -> level -> zero-thrust."""

    def __init__(self):
        super().__init__('attitude_setpoint_node')
        self.declare_parameter('setpoint_topic', '/quadcopter/attitude_setpoint')
        self.declare_parameter('mode_topic', '/quadcopter/control_mode')
        self.declare_parameter('hover_duration', 5.0)
        self.declare_parameter('step_duration', 5.0)
        self.declare_parameter('zero_thrust_duration', 5.0)
        self.declare_parameter('roll_step_deg', 15.0)
        self.declare_parameter('pitch_step_deg', -10.0)
        self.declare_parameter('yaw_step_deg', 30.0)
        self.declare_parameter('publish_rate', 50.0)

        self._hover_duration = self.get_parameter('hover_duration').value
        self._step_duration = self.get_parameter('step_duration').value
        self._zero_duration = self.get_parameter('zero_thrust_duration').value
        self._roll_step = self.get_parameter('roll_step_deg').value
        self._pitch_step = self.get_parameter('pitch_step_deg').value
        self._yaw_step = self.get_parameter('yaw_step_deg').value
        self._rate = self.get_parameter('publish_rate').value

        self._sim_time = 0.0
        self._start_time = None
        self._finished = False

        self._setpoint_pub = self.create_publisher(
            QuaternionStamped,
            self.get_parameter('setpoint_topic').value, 10)
        self._mode_pub = self.create_publisher(
            String, self.get_parameter('mode_topic').value, 10)
        self._clock_sub = self.create_subscription(
            Clock, '/clock', self._on_clock, 10)
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            'Setpoint driver started: hover={:.0f}s steps={:.0f}s '
            'zero={:.0f}s (roll {:.0f} deg, pitch {:.0f} deg, yaw {:.0f} deg)'
            .format(self._hover_duration, self._step_duration,
                    self._zero_duration, self._roll_step, self._pitch_step,
                    self._yaw_step))

    def _on_clock(self, msg):
        self._sim_time = msg.clock.sec + 1e-9 * msg.clock.nanosec

    def _schedule(self, t):
        """Return (roll, pitch, yaw, mode) for simulation time t."""
        hover = self._hover_duration
        step = self._step_duration
        if t < hover:
            return (0.0, 0.0, 0.0, 'hover')
        if t < hover + 1 * step:
            return (self._roll_step, 0.0, 0.0, 'hover')
        if t < hover + 2 * step:
            return (0.0, self._pitch_step, 0.0, 'hover')
        if t < hover + 3 * step:
            return (0.0, 0.0, self._yaw_step, 'hover')
        if t < hover + 4 * step:
            return (0.0, 0.0, 0.0, 'hover')
        if t < hover + 4 * step + self._zero_duration:
            return (0.0, 0.0, 0.0, 'zero_thrust')
        return None

    def _tick(self):
        if self._finished:
            return
        if self._start_time is None:
            self._start_time = self._sim_time
            self.get_logger().info('schedule starts at t={:.3f}s'.format(
                self._start_time))
        t = self._sim_time - self._start_time
        step = self._schedule(t)
        if step is None:
            self.get_logger().info('schedule finished at t={:.3f}s'.format(
                self._sim_time))
            self._finished = True
            self.destroy_timer(self._timer)
            threading.Thread(target=rclpy.shutdown, daemon=True).start()
            return

        roll, pitch, yaw, mode = step
        q_msg = QuaternionStamped()
        q_msg.header.stamp = self.get_clock().now().to_msg()
        q_msg.header.frame_id = 'world'
        q = quaternion_from_rpy(
            roll * 3.141592653589793 / 180.0,
            pitch * 3.141592653589793 / 180.0,
            yaw * 3.141592653589793 / 180.0)
        q_msg.quaternion.x = q[1]
        q_msg.quaternion.y = q[2]
        q_msg.quaternion.z = q[3]
        q_msg.quaternion.w = q[0]
        self._setpoint_pub.publish(q_msg)

        mode_msg = String()
        mode_msg.data = mode
        self._mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AttitudeSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
