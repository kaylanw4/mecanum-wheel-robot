from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add config and launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kaylan Wang',
    maintainer_email='kaylanwang5@gmail.com',
    description='Teleoperation nodes for Yahboom mecanum wheel robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yahboom_joystick = robot_teleop.yahboom_joystick:main',
        ],
    },
)