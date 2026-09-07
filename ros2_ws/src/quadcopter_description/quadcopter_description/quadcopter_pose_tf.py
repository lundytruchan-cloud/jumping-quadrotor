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

"""Publish the Gazebo model pose as a ``world -> base_link`` transform."""

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class QuadcopterPoseTf(Node):
    """Bridge a bridged PoseStamped topic into the TF tree."""

    def __init__(self):
        super().__init__('quadcopter_pose_tf')
        self.declare_parameter('pose_topic', '/quadcopter/pose')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame', 'base_link')

        self._parent_frame = self.get_parameter('parent_frame').value
        self._child_frame = self.get_parameter('child_frame').value
        self._broadcaster = TransformBroadcaster(self)
        self._subscriber = self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            self._on_pose,
            10,
        )
        self.get_logger().info(
            'Publishing {} -> {} from {}'.format(
                self._parent_frame,
                self._child_frame,
                self.get_parameter('pose_topic').value,
            )
        )

    def _on_pose(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z
        transform.transform.rotation = msg.pose.orientation
        self._broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = QuadcopterPoseTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
