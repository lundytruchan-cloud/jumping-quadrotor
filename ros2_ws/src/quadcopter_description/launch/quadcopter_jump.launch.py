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

"""Gazebo + RViz + low-level attitude control + high-level jump control."""

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
    args = [
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare(package_name), 'worlds',
                'quadcopter_world.sdf'])),
        DeclareLaunchArgument('namespace', default_value='quadcopter'),
        DeclareLaunchArgument('spawn_z', default_value='0.22'),
        DeclareLaunchArgument('drop_height', default_value='0.8'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('trajectory', default_value='spot'),
        DeclareLaunchArgument('desired_height', default_value='0.6'),
        DeclareLaunchArgument('circle_radius', default_value='0.5'),
        DeclareLaunchArgument('circle_omega', default_value='0.45'),
        DeclareLaunchArgument('circle_phase0', default_value='0.0'),
        DeclareLaunchArgument('step_targets', default_value='0,0;0.6,0'),
        DeclareLaunchArgument('step_hold', default_value='2.0'),
        DeclareLaunchArgument('num_hops', default_value='0'),
        DeclareLaunchArgument('duration', default_value='0.0'),
        DeclareLaunchArgument('pre_landing_lead', default_value='0.7'),
        DeclareLaunchArgument('mu', default_value='0.8'),
        DeclareLaunchArgument('spin_up_comp', default_value='0.05'),
        DeclareLaunchArgument('max_tilt_deg', default_value='10.0'),
        DeclareLaunchArgument('tilt_scale', default_value='1.0'),
        DeclareLaunchArgument('fb_gain', default_value='0.0'),
        DeclareLaunchArgument('fb_clamp', default_value='0.25'),
        DeclareLaunchArgument('fixed_tilt_deg', default_value='0.0'),
        DeclareLaunchArgument('max_speed', default_value='1.5'),
        DeclareLaunchArgument('max_step', default_value='0.25'),
        DeclareLaunchArgument('rotation_time', default_value='0.25'),
        DeclareLaunchArgument('ramp_hops', default_value='3'),
        DeclareLaunchArgument('adaptive_scale', default_value='true'),
        DeclareLaunchArgument('log_file', default_value=''),
        DeclareLaunchArgument('attitude_log', default_value=''),
        DeclareLaunchArgument('leg_log', default_value=''),
    ]

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
            'altitude_hold': False,
            'kd_rate': 0.0,
            'kp_rate': 1.2e-3,
            'ki_rate': 1.0e-3,
            'kp_att': 10.0,
            'log_file': LaunchConfiguration('attitude_log'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    jump_controller = Node(
        package=package_name,
        executable='jump_control_node',
        parameters=[{
            'trajectory': LaunchConfiguration('trajectory'),
            'desired_height': LaunchConfiguration('desired_height'),
            'circle_radius': LaunchConfiguration('circle_radius'),
            'circle_omega': LaunchConfiguration('circle_omega'),
            'circle_phase0': LaunchConfiguration('circle_phase0'),
            'step_targets': LaunchConfiguration('step_targets'),
            'step_hold': LaunchConfiguration('step_hold'),
            'num_hops': LaunchConfiguration('num_hops'),
            'duration': LaunchConfiguration('duration'),
            'pre_landing_lead': LaunchConfiguration('pre_landing_lead'),
            'mu': LaunchConfiguration('mu'),
            'spin_up_comp': LaunchConfiguration('spin_up_comp'),
            'max_tilt_deg': LaunchConfiguration('max_tilt_deg'),
            'tilt_scale': LaunchConfiguration('tilt_scale'),
            'fb_gain': LaunchConfiguration('fb_gain'),
            'fb_clamp': LaunchConfiguration('fb_clamp'),
            'fixed_tilt_deg': LaunchConfiguration('fixed_tilt_deg'),
            'max_speed': LaunchConfiguration('max_speed'),
            'max_step': LaunchConfiguration('max_step'),
            'rotation_time': LaunchConfiguration('rotation_time'),
            'ramp_hops': LaunchConfiguration('ramp_hops'),
            'adaptive_scale': LaunchConfiguration('adaptive_scale'),
            'log_file': log_file,
            'use_sim_time': True,
        }],
        output='screen',
    )

    phase_node = Node(
        package=package_name,
        executable='leg_phase_node',
        parameters=[{
            'log_file': LaunchConfiguration('leg_log'),
            'duration': 0.0,
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        *args,
        sim_launch,
        imu_bridge,
        motor_bridge,
        controller,
        jump_controller,
        phase_node,
    ])
