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

"""Hybrid-locomotion phase node (Gazebo leg state -> phase + CSV)."""

import csv
import threading

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Float32Array
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from .phase_machine import PhaseStateMachine


class LegPhaseNode(Node):
    """Track the hybrid-locomotion phase and log leg telemetry."""

    def __init__(self):
        super().__init__('leg_phase_node')
        self.declare_parameter('leg_state_topic', '/quadcopter/leg_state')
        self.declare_parameter('phase_topic', '/quadcopter/phase')
        self.declare_parameter('log_file', '')
        self.declare_parameter('duration', 0.0)
        self.declare_parameter('foot_threshold', 0.03)
        self.declare_parameter('compression_eps', 0.002)

        self._leg_state_topic = self.get_parameter('leg_state_topic').value
        self._phase_topic = self.get_parameter('phase_topic').value
        self._log_file = self.get_parameter('log_file').value
        self._duration = self.get_parameter('duration').value

        self._machine = PhaseStateMachine(
            foot_threshold=self.get_parameter('foot_threshold').value,
            compression_eps=self.get_parameter('compression_eps').value,
        )
        self._sim_time = 0.0
        self._start_time = None
        self._last_phase = None
        self._sample_count = 0
        self._finished = False

        self._csv_writer = None
        self._csv_file = None
        if self._log_file:
            self._csv_file = open(self._log_file, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'sim_time', 'foot_z', 'leg_q', 'leg_qdot',
                'spring_force', 'phase',
            ])

        self._clock_sub = self.create_subscription(
            Clock, '/clock', self._on_clock, 10)
        self._leg_state_sub = self.create_subscription(
            Float32Array, self._leg_state_topic, self._on_leg_state, 100)
        self._phase_pub = self.create_publisher(String, self._phase_topic, 10)
        self._timer = self.create_timer(0.1, self._check_finished)
        self.get_logger().info(
            'Phase node started: topic={} log={}'.format(
                self._leg_state_topic, self._log_file or '(none)'),
        )

    def _on_clock(self, msg):
        self._sim_time = msg.clock.sec + 1e-9 * msg.clock.nanosec
        if self._start_time is None:
            self._start_time = self._sim_time

    def _on_leg_state(self, msg):
        if len(msg.data) < 5 or self._finished:
            return
        t = float(msg.data[0])
        foot_z = float(msg.data[4])
        q = float(msg.data[1])
        q_dot = float(msg.data[2])
        force = float(msg.data[3])
        phase = self._machine.update(t, foot_z, q, q_dot)
        self._sample_count += 1

        if phase != self._last_phase:
            self._last_phase = phase
            phase_msg = String()
            phase_msg.data = phase.name
            self._phase_pub.publish(phase_msg)
            self.get_logger().info(
                'phase -> {} at t={:.4f}s (foot_z={:.4f}, q={:.4f})'
                .format(phase.name, t, foot_z, q),
            )

        if self._csv_writer is not None:
            self._csv_writer.writerow([
                '{:.9f}'.format(t),
                '{:.9f}'.format(foot_z),
                '{:.9f}'.format(q),
                '{:.9f}'.format(q_dot),
                '{:.9f}'.format(force),
                phase.name,
            ])
            if self._sample_count % 500 == 0:
                self._csv_file.flush()

    def _check_finished(self):
        if self._finished or self._start_time is None:
            return
        elapsed = self._sim_time - self._start_time
        if self._duration > 0.0 and elapsed >= self._duration:
            self.get_logger().info(
                'Recorded {} samples in {:.2f}s sim time'.format(
                    self._sample_count, elapsed),
            )
            self._finish()

    def _finish(self):
        self._finished = True
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_writer = None
        self.destroy_timer(self._timer)
        self.get_logger().info('Phase node finished')
        threading.Thread(target=rclpy.shutdown, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = LegPhaseNode()
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
