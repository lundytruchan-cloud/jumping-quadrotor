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

"""Node-level test for the low-level control modes used by the jumper."""

import time

from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped, QuaternionStamped
import pytest
from quadcopter_description.attitude_control_node import AttitudeControlNode
from quadcopter_description.attitude_math import quaternion_from_rpy
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String


@pytest.fixture(scope='module', autouse=True)
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wait_for(node, containers, timeout=3.0):
    """Spin the node until all containers are non-empty or the timeout hits."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(len(c) > 0 for c in containers):
            return True
    return False


def test_attitude_only_mode_commands_torque_without_thrust():
    node = AttitudeControlNode()
    node.set_parameters([
        rclpy.parameter.Parameter('altitude_hold', value=False),
    ])
    received = []
    diag = []
    sub = node.create_subscription(
        Actuators, '/quadcopter/motor_speed',
        lambda msg: received.append(msg), 10)
    diag_sub = node.create_subscription(
        Float32MultiArray, '/quadcopter/control_state',
        lambda msg: diag.append(msg), 10)

    clock = Clock()
    clock.clock.sec = 10
    node._on_clock(clock)

    imu = Imu()
    imu.orientation.w = 1.0
    node._on_imu(imu)

    setpoint = QuaternionStamped()
    q = quaternion_from_rpy(0.3, 0.0, 0.0)
    setpoint.quaternion.w = q[0]
    setpoint.quaternion.x = q[1]
    setpoint.quaternion.y = q[2]
    setpoint.quaternion.z = q[3]
    node._on_setpoint(setpoint)

    node._on_mode(String(data='attitude_only'))
    assert node._mode == 'attitude_only'

    pose = PoseStamped()
    pose.pose.position.z = 0.3
    node._on_pose(pose)

    node._tick()
    assert _wait_for(node, [received, diag])
    speeds = received[0].velocity
    # Attitude error must produce motor activity even with zero thrust.
    assert max(abs(s) for s in speeds) > 0.0
    assert diag[0].data[6] == pytest.approx(0.0, abs=1e-9)

    node.destroy_subscription(sub)
    node.destroy_subscription(diag_sub)
    node.destroy_node()


def test_attitude_only_mode_limits_net_thrust():
    node = AttitudeControlNode()
    diag = []
    diag_sub = node.create_subscription(
        Float32MultiArray, '/quadcopter/control_state',
        lambda msg: diag.append(msg), 10)

    clock = Clock()
    clock.clock.sec = 10
    node._on_clock(clock)
    imu = Imu()
    imu.orientation.w = 1.0
    node._on_imu(imu)

    setpoint = QuaternionStamped()
    q = quaternion_from_rpy(0.8, 0.4, 0.0)
    setpoint.quaternion.w = q[0]
    setpoint.quaternion.x = q[1]
    setpoint.quaternion.y = q[2]
    setpoint.quaternion.z = q[3]
    node._on_setpoint(setpoint)
    node._on_mode(String(data='attitude_only'))

    pose = PoseStamped()
    pose.pose.position.z = 0.3
    node._on_pose(pose)
    node._tick()
    assert _wait_for(node, [diag])
    # A large attitude error must not turn the ballistic phase into a
    # thruster: the net thrust stays bounded.
    assert diag[0].data[7] <= 0.25 + 1e-4

    node.destroy_subscription(diag_sub)
    node.destroy_node()


def test_attitude_only_mode_ignores_yaw_error():
    node = AttitudeControlNode()
    diag = []
    diag_sub = node.create_subscription(
        Float32MultiArray, '/quadcopter/control_state',
        lambda msg: diag.append(msg), 10)

    clock = Clock()
    clock.clock.sec = 10
    node._on_clock(clock)
    imu = Imu()
    imu.orientation.w = 1.0
    imu.angular_velocity.z = 5.0  # yaw rate must also be ignored
    node._on_imu(imu)

    setpoint = QuaternionStamped()
    q = quaternion_from_rpy(0.0, 0.0, 0.5)  # pure yaw error
    setpoint.quaternion.w = q[0]
    setpoint.quaternion.x = q[1]
    setpoint.quaternion.y = q[2]
    setpoint.quaternion.z = q[3]
    node._on_setpoint(setpoint)
    node._on_mode(String(data='attitude_only'))

    pose = PoseStamped()
    pose.pose.position.z = 0.3
    node._on_pose(pose)
    node._tick()
    assert _wait_for(node, [diag])
    # The hopping cycle does not need yaw tracking; it must not waste the
    # ballistic torque budget on yaw (which would create net thrust).
    assert abs(diag[0].data[5]) < 1e-9  # tau_z

    node.destroy_subscription(diag_sub)
    node.destroy_node()


def test_zero_thrust_mode_stops_all_motors():
    node = AttitudeControlNode()
    received = []
    sub = node.create_subscription(
        Actuators, '/quadcopter/motor_speed',
        lambda msg: received.append(msg), 10)

    clock = Clock()
    clock.clock.sec = 10
    node._on_clock(clock)
    imu = Imu()
    imu.orientation.w = 1.0
    node._on_imu(imu)
    node._on_mode(String(data='zero_thrust'))

    node._tick()
    assert _wait_for(node, [received])
    assert all(abs(s) < 1e-9 for s in received[0].velocity)

    node.destroy_subscription(sub)
    node.destroy_node()
