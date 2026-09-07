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

"""Gazebo + RViz + low-level attitude controller (task 6)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = 'quadcopter_description'


def generate_launch_description():
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name), 'worlds',
            'quadcopter_world.sdf']),
        description='Gazebo world SDF file',
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='quadcopter',
        description='Model name used to spawn and for topic names',
    )
    declare_spawn_z = DeclareLaunchArgument(
        'spawn_z', default_value='0.22',
        description='Body height so the central leg rests on the ground',
    )
    declare_drop_height = DeclareLaunchArgument(
        'drop_height', default_value='0.0',
        description='Initial foot height above the ground for a drop',
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Whether to start the Gazebo GUI',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz',
    )
    declare_log_file = DeclareLaunchArgument(
        'log_file', default_value='',
        description='CSV path for attitude control telemetry',
    )

    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    spawn_z = LaunchConfiguration('spawn_z')
    drop_height = LaunchConfiguration('drop_height')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    log_file = LaunchConfiguration('log_file')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package_name), 'launch',
            'quadcopter_sim.launch.py'])),
        launch_arguments=[
            ('world', world),
            ('namespace', namespace),
            ('spawn_z', spawn_z),
            ('drop_height', drop_height),
            ('gui', gui),
            ('rviz', rviz),
        ],
    )

    imu_topic = (
        '/world/quadcopter_world/model/quadcopter/'
        'link/base_link/sensor/imu_sensor/imu'
    )
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '{}@sensor_msgs/msg/Imu[gz.msgs.IMU'.format(imu_topic),
        ],
        remappings=[(imu_topic, '/quadcopter/imu')],
        output='screen',
    )

    motor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/quadcopter/gazebo/command/motor_speed'
            '@actuator_msgs/msg/Actuators]gz.msgs.Actuators',
        ],
        remappings=[
            ('/quadcopter/gazebo/command/motor_speed',
             '/quadcopter/motor_speed'),
        ],
        output='screen',
    )

    controller = Node(
        package=package_name,
        executable='attitude_control_node',
        parameters=[{
            'log_file': log_file,
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_world,
        declare_namespace,
        declare_spawn_z,
        declare_drop_height,
        declare_gui,
        declare_rviz,
        declare_log_file,
        sim_launch,
        imu_bridge,
        motor_bridge,
        controller,
    ])
