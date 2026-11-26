from setuptools import setup
import os
from glob import glob

package_name = 'drone_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='howe12',
    maintainer_email='howe12@users.noreply.github.com',
    description='我的仿真无人机开发包：PX4 SITL + ROS2 Humble offboard 控制',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_demo = drone_sim.offboard_demo:main',
            'waypoint_mission = drone_sim.waypoint_mission:main',
            'formation_follower = drone_sim.formation_follower:main',
        ],
    },
)
