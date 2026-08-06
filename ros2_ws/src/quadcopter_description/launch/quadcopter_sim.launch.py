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

"""Launch Gazebo (server + GUI), spawn the quadcopter and start RViz."""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = 'quadcopter_description'


def _convert_and_spawn(context):
    """Expand xacro to URDF, convert to SDF and create the model in Gazebo."""
    package_share = get_package_share_directory(package_name)
    xacro_path = os.path.join(package_share, 'urdf', 'quadcopter.urdf.xacro')
    spawn_z = float(context.launch_configurations['spawn_z'])
    namespace = context.launch_configurations['namespace']

    with tempfile.TemporaryDirectory(prefix='quadcopter_spawn_') as tmp:
        urdf_path = os.path.join(tmp, 'quadcopter.urdf')
        xacro = subprocess.run(
            ['xacro', xacro_path, '-o', urdf_path],
            capture_output=True,
            text=True,
        )
        if xacro.returncode != 0:
            raise RuntimeError(
                'xacro failed: {}'.format(xacro.stderr.strip()))

        converted = subprocess.run(
            ['gz', 'sdf', '-p', urdf_path],
            capture_output=True,
            text=True,
        )
        if converted.returncode != 0:
            raise RuntimeError(
                'gz sdf conversion failed: {}'
                .format(converted.stderr.strip()))
        sdf_string = converted.stdout

    command = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-string', sdf_string,
        '-world', 'quadcopter_world',
        '-name', namespace,
        '-x', '0', '-y', '0', '-z', str(spawn_z),
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            'Spawn failed: {}'.format(result.stderr.strip()))
    return []


def _robot_state_publisher(context):
    """Expand the xacro model and start robot_state_publisher with it."""
    package_share = get_package_share_directory(package_name)
    xacro_path = os.path.join(package_share, 'urdf', 'quadcopter.urdf.xacro')
    expanded = subprocess.run(
        ['xacro', xacro_path], capture_output=True, text=True)
    if expanded.returncode != 0:
        raise RuntimeError(
            'xacro failed: {}'.format(expanded.stderr.strip()))
    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': expanded.stdout,
            'use_sim_time': True,
        }],
    )]


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
        'spawn_z', default_value='0.041',
        description='Initial height so the legs rest on the ground',
    )
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Whether to start the Gazebo GUI',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to start RViz',
    )

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world, '-v', '3'],
        output='screen',
    )
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-r'],
        output='screen',
        condition=IfCondition(gui),
    )

    spawn_entity = TimerAction(
        period=3.0,
        actions=[OpaqueFunction(function=_convert_and_spawn)],
    )

    robot_state_publisher = OpaqueFunction(
        function=_robot_state_publisher)

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/quadcopter_world/clock'
            '@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/world/quadcopter_world/clock', '/clock'),
        ],
        output='screen',
    )
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/quadcopter_world/model/quadcopter/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/quadcopter_world/model/quadcopter/joint_state',
             '/joint_states'),
        ],
        output='screen',
    )
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/quadcopter/pose'
            '@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
        ],
        remappings=[
            ('/model/quadcopter/pose', '/quadcopter/pose'),
        ],
        output='screen',
    )

    pose_tf = Node(
        package=package_name,
        executable='quadcopter_pose_tf',
        parameters=[{
            'pose_topic': '/quadcopter/pose',
            'parent_frame': 'world',
            'child_frame': 'base_link',
            'use_sim_time': True,
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare(package_name), 'config',
                'quadcopter.rviz']),
        ],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_world,
        declare_namespace,
        declare_spawn_z,
        declare_gui,
        declare_rviz,
        gazebo_server,
        TimerAction(period=1.0, actions=[gazebo_gui]),
        spawn_entity,
        clock_bridge,
        joint_state_bridge,
        pose_bridge,
        robot_state_publisher,
        pose_tf,
        rviz_node,
    ])
