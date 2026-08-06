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

"""Validate the generated quadcopter URDF against design parameters."""

import os
import subprocess
import xml.etree.ElementTree as ET


ROBOT_DIR = os.path.join(os.path.dirname(__file__), '..', 'urdf')
XACRO_PATH = os.path.join(ROBOT_DIR, 'quadcopter.urdf.xacro')


def expand_xacro():
    """Return the root element of the xacro-expanded URDF."""
    result = subprocess.run(
        ['xacro', XACRO_PATH], capture_output=True, text=True, check=True)
    return ET.fromstring(result.stdout)


def test_robot_structure():
    root = expand_xacro()
    assert root.get('name') == 'quadcopter'
    links = root.findall('link')
    joints = root.findall('joint')
    assert len(links) == 5
    assert len(joints) == 4
    for index in range(4):
        joint = root.find('joint[@name="prop{}_joint"]'.format(index))
        assert joint is not None
        assert joint.get('type') == 'continuous'
        assert joint.find('parent').get('link') == 'base_link'
        assert joint.find('child').get('link') == 'prop{}_link'.format(index)
        axis = joint.find('axis')
        assert axis is not None
        assert axis.get('xyz') == '0 0 1'


def test_crazyflie_mass():
    root = expand_xacro()
    total_mass = sum(
        float(link.find('inertial/mass').get('value'))
        for link in root.findall('link'))
    assert abs(total_mass - 0.027) < 1e-6


def test_crazyflie_geometry():
    root = expand_xacro()
    base_inertial = root.find('link[@name="base_link"]/inertial')
    assert base_inertial is not None
    inertia = base_inertial.find('inertia')
    for axis in ('ixx', 'iyy', 'izz'):
        assert float(inertia.get(axis)) > 0.0
    joint = root.find('joint[@name="prop0_joint"]')
    origin = joint.find('origin').get('xyz').split()
    motor_xy = float(origin[0])
    assert abs(motor_xy - 0.032527) < 1e-4


def test_gazebo_plugins_present():
    root = expand_xacro()
    motor_plugins = [
        plugin for plugin in root.findall('gazebo/plugin')
        if 'MulticopterMotorModel' in plugin.get('name')]
    joint_plugins = [
        plugin for plugin in root.findall('gazebo/plugin')
        if 'JointStatePublisher' in plugin.get('name')]
    assert len(motor_plugins) == 4
    assert len(joint_plugins) == 1
    assert len(joint_plugins[0].findall('joint_name')) == 4
