#!/usr/bin/env python3
"""Flake8 linting test for robot_pointcloud_preprocessing package."""

import subprocess
import sys


def test_flake8():
    """Test that the package follows PEP 8 style guidelines."""
    try:
        result = subprocess.run(
            ['python3', '-m', 'flake8', 'robot_pointcloud_preprocessing/'],
            cwd='/home/kaylanw4/ros2_ws/src/robot_pointcloud_preprocessing',
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0:
            print("Flake8 issues found:")
            print(result.stdout)
            print(result.stderr)
            
        assert result.returncode == 0, "Flake8 found style issues"
        
    except FileNotFoundError:
        print("flake8 not available, skipping test")


if __name__ == '__main__':
    test_flake8()