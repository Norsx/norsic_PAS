#!/usr/bin/env python3
"""
setup.py za student_assignment_02 ROS 2 Python paket
"""

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'student_assignment_02'

# Pronađi sve packages u current direktoriju
packages = find_packages(where='.', include=['student_assignment_02*'])

# Ako nema packages, error
if not packages:
    raise RuntimeError(
        f"ERROR: No packages found!\n"
        f"Make sure {package_name}/__init__.py exists"
    )

# Pronađi sve data files
data_files = []

# Resource file
data_files.append(('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]))

# package.xml
data_files.append(('share/' + package_name, ['package.xml']))

# Launch files
if os.path.isdir('launch'):
    launch_files = glob('launch/*.py')
    if launch_files:
        data_files.append(('share/' + package_name + '/launch', launch_files))

# Config files
if os.path.isdir('config'):
    config_files = glob('config/*.yaml') + glob('config/*.rviz')
    if config_files:
        data_files.append(('share/' + package_name + '/config', config_files))

# World files
if os.path.isdir('world'):
    world_files = glob('world/*.world') + glob('world/*.png')
    if world_files:
        data_files.append(('share/' + package_name + '/world', world_files))
    # Include files
    include_files = glob('world/include/*.inc')
    if include_files:
        data_files.append(('share/' + package_name + '/world/include', include_files))

# Mapped maps
if os.path.isdir('mapped_maps'):
    for root, dirs, files in os.walk('mapped_maps'):
        if files:
            rel_path = os.path.relpath(root, '.')
            dest = os.path.join('share', package_name, rel_path)
            src_files = [os.path.join(rel_path, f) for f in files]
            data_files.append((dest, src_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khartl',
    maintainer_email='kh239762@fsb.hr',
    description='ROS2 Python package for autonomous system mapping and path planning using A* and D* algorithms',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a_star_path_planner = student_assignment_02.a_star_path_planner:main',
            'd_star_path_planner = student_assignment_02.d_star_path_planner:main',
            'map_republisher = student_assignment_02.map_republisher:main',
            'path_planning_node = student_assignment_02.path_planning_node:main',
            'goal_navigation_node = student_assignment_02.goal_navigation_node:main',
            'path_follower = student_assignment_02.path_follower_node:main',
            'nav2_adapter = student_assignment_02.nav2_adapter:main',
        ],
    },
)
