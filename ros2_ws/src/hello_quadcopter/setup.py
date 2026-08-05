from glob import glob

import os

from setuptools import find_packages, setup

package_name = 'hello_quadcopter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='larry',
    maintainer_email='310690260+lundytruchan-cloud@users.noreply.github.com',
    description='Minimal hello node and launch file for the quadcopter',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_quadcopter_node = '
            'hello_quadcopter.hello_quadcopter_node:main',
        ],
    },
)
