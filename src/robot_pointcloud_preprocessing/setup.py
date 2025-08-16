from setuptools import setup
import os
from glob import glob

package_name = 'robot_pointcloud_preprocessing'

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
    maintainer='Robot Developer',
    maintainer_email='robot@yahboom.com',
    description='Point cloud preprocessing pipeline for ZED2i camera data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_preprocessor = robot_pointcloud_preprocessing.pointcloud_preprocessor:main',
        ],
    },
)