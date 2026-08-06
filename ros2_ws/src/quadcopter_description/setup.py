from glob import glob

import os

from setuptools import find_packages, setup

package_name = 'quadcopter_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larry',
    maintainer_email='310690260+lundytruchan-cloud@users.noreply.github.com',
    description='Crazyflie 2.1 inspired quadcopter description package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'quadcopter_pose_tf = '
            'quadcopter_description.quadcopter_pose_tf:main',
            'quadcopter_demo_node = '
            'quadcopter_description.quadcopter_demo_node:main',
            'leg_phase_node = '
            'quadcopter_description.leg_phase_node:main',
            'attitude_control_node = '
            'quadcopter_description.attitude_control_node:main',
            'attitude_setpoint_node = '
            'quadcopter_description.attitude_setpoint_node:main',
            'jump_control_node = '
            'quadcopter_description.jump_control_node:main',
        ],
    },
)
