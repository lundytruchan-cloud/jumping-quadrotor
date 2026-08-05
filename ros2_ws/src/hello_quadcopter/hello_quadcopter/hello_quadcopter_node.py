# Copyright 2026 larry
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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def build_status_message(tick):
    """Build the status message text for the given tick number."""
    return f'Hello from quadcopter! tick={tick}'


class HelloQuadcopterNode(Node):
    """Minimal node that periodically publishes a status message."""

    def __init__(self):
        super().__init__('hello_quadcopter_node')
        self._tick = 0
        self._publisher = self.create_publisher(
            String, 'hello_quadcopter/status', 10
        )
        self.create_timer(1.0, self._publish_status)

    def _publish_status(self):
        self._tick += 1
        message = String()
        message.data = build_status_message(self._tick)
        self._publisher.publish(message)
        self.get_logger().info('Publishing: %s' % message.data)


def main(args=None):
    """Run the node until interrupted."""
    rclpy.init(args=args)
    node = HelloQuadcopterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
