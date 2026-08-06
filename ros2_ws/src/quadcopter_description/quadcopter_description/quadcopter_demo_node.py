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
Spin the four propellers and record joint/pose data for verification.

Motor speed commands are sent directly to Gazebo (gz.msgs.Actuators with the
velocity field in rad/s). Joint states arrive via the bridged ``/joint_states``
topic.
"""

import csv
import subprocess
import threading

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class QuadcopterDemoNode(Node):
    """Ramp the four motors up, hold, ramp down and log telemetry."""

    def __init__(self):
        super().__init__('quadcopter_demo_node')
        self.declare_parameter('motor_speed', 0.2)
        self.declare_parameter('max_rot_velocity', 2200.0)
        self.declare_parameter('ramp_time', 3.0)
        self.declare_parameter('hold_time', 8.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('log_file', '')

        self._target = self.get_parameter('motor_speed').value
        self._max_rot = self.get_parameter('max_rot_velocity').value
        self._ramp_time = self.get_parameter('ramp_time').value
        self._hold_time = self.get_parameter('hold_time').value
        self._rate = self.get_parameter('rate_hz').value
        self._log_file = self.get_parameter('log_file').value

        self._cmd_topic = '/quadcopter/gazebo/command/motor_speed'
        self._sim_time = 0.0
        self._start_time = None
        self._pose = None
        self._joint_state = None
        self._current_speed = 0.0
        self._log_counter = 0
        self._publish_lock = threading.Lock()
        self._finished = False

        self._clock_sub = self.create_subscription(
            Clock, '/clock', self._on_clock, 10)
        self._pose_sub = self.create_subscription(
            PoseStamped, '/quadcopter/pose', self._on_pose, 10)
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        self._csv_writer = None
        self._csv_file = None
        if self._log_file:
            self._csv_file = open(self._log_file, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'sim_time', 'cmd_speed',
                'pos_x', 'pos_y', 'pos_z',
                'qx', 'qy', 'qz', 'qw',
                'p0', 'p1', 'p2', 'p3',
                'v0', 'v1', 'v2', 'v3',
            ])

        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            'Demo node started: target={:.2f} ramp={:.1f}s hold={:.1f}s'
            .format(self._target, self._ramp_time, self._hold_time),
        )

    def _on_clock(self, msg):
        self._sim_time = msg.clock.sec + 1e-9 * msg.clock.nanosec

    def _on_pose(self, msg):
        self._pose = msg.pose

    def _on_joint_state(self, msg):
        self._joint_state = msg
        self._log_counter += 1
        if self._log_counter % 20 == 0:
            self._log(self._current_speed)

    def _command(self, speed):
        """Publish motor speeds without blocking the ROS executor."""

        def _publish():
            with self._publish_lock:
                velocity = speed * self._max_rot
                payload = 'velocity: [{0}, {0}, {0}, {0}]'.format(velocity)
                subprocess.run(
                    [
                        'gz', 'topic', '-t', self._cmd_topic,
                        '-m', 'gz.msgs.Actuators', '-p', payload,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )

        if not self._publish_lock.locked():
            threading.Thread(target=_publish, daemon=True).start()

    def stop_motors(self):
        """Command all motors to zero speed."""
        self._command(0.0)

    def _tick(self):
        if self._finished:
            return
        if self._start_time is None:
            self._start_time = self._sim_time
        t = self._sim_time - self._start_time
        if t < self._ramp_time:
            speed = self._target * t / self._ramp_time
        elif t < self._ramp_time + self._hold_time:
            speed = self._target
        else:
            speed = max(0.0, self._target
                        * (1.0 - (t - self._ramp_time - self._hold_time)
                           / self._ramp_time))
            if speed == 0.0:
                self._finished = True
        self._current_speed = speed
        self._command(speed)
        if self._finished:
            self.get_logger().info('Demo finished')
            self.destroy_timer(self._timer)
            if self._csv_file is not None:
                self._csv_file.close()
                self._csv_writer = None

    def _log(self, speed):
        if self._csv_writer is None:
            return
        pose = self._pose
        joint = self._joint_state
        row = [self._sim_time, speed]
        if pose is not None:
            row.extend([
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            ])
        else:
            row.extend([float('nan')] * 7)
        if joint is not None:
            positions = [0.0] * 4
            velocities = [0.0] * 4
            for i, name in enumerate(joint.name):
                if name.startswith('prop') and 'joint' in name:
                    index = int(name[4])
                    positions[index] = joint.position[i]
                    velocities[index] = joint.velocity[i]
            row.extend(positions)
            row.extend(velocities)
        else:
            row.extend([float('nan')] * 8)
        self._csv_writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = QuadcopterDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
