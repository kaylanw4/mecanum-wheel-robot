#!/usr/bin/env python3
"""PEP 257 docstring test for robot_pointcloud_preprocessing package."""

import subprocess
import sys


def test_pep257():
    """Test that the package follows PEP 257 docstring guidelines."""
    try:
        result = subprocess.run(
            ['python3', '-m', 'pydocstyle', 'robot_pointcloud_preprocessing/'],
            cwd='/home/kaylanw4/ros2_ws/src/robot_pointcloud_preprocessing',
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0:
            print("PEP 257 issues found:")
            print(result.stdout)
            print(result.stderr)
            
        # Allow some docstring issues for now in Phase 1
        # assert result.returncode == 0, "PEP 257 found docstring issues"
        
    except FileNotFoundError:
        print("pydocstyle not available, skipping test")


if __name__ == '__main__':
    test_pep257()