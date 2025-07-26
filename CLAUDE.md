# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 Humble mecanum wheel robot project running on NVIDIA Jetson Orin Nano with Yahboom hardware components. The project includes both real hardware control and Gazebo simulation capabilities for autonomous robotics development.

**Hardware**: NVIDIA Jetson Orin Nano 8GB, Yahboom Robot Expansion Board V3.0, mecanum wheels, DC motors with encoders, PS4 controller
**Software Stack**: ROS2 Humble, Gazebo Fortress, ros2_controllers, Ubuntu 22.04 LTS

## Package Architecture

- **robot_bringup**: Main launch files and system configuration (C++ package with ament_cmake)
- **robot_description**: URDF models and robot description files (C++ package with ament_cmake) 
- **robot_hardware**: Hardware drivers and interfaces (Python package with setuptools)
- **robot_teleop**: Teleoperation and joystick control (Python package with setuptools)
- **mecanum_robot_sim**: Simulation-specific components (C++ package with ament_cmake)

## Common Development Commands

### Build and Setup
```bash
# Build entire workspace
colcon build --symlink-install

# Build specific package
colcon build --packages-select robot_bringup

# Build with debug info
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source workspace (add to ~/.bashrc)
source install/setup.bash
```

### Testing
```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select robot_hardware

# Run Python linting tests (flake8, copyright, pep257)
pytest src/robot_hardware/test/
pytest src/robot_teleop/test/

# View test results
colcon test-result --all
```

### Launch System
```bash
# Launch joystick control
ros2 launch robot_bringup joystick.launch.py

# Launch Gazebo simulation
ros2 launch robot_bringup gazebo_sim.launch.py

# Launch with arguments
ros2 launch robot_bringup gazebo_sim.launch.py use_sim_time:=true robot_name:=my_robot
```

### Development Tools
```bash
# Check ROS2 system
ros2 doctor

# List all packages
colcon list

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Check package dependencies
rosdep check --from-paths src --ignore-src
```

## Key Configuration Files

- `src/robot_bringup/config/joystick.yaml`: PS4 controller configuration
- `src/robot_bringup/config/ros2_controllers.yaml`: Robot controller parameters
- `src/robot_bringup/urdf/yahboomcar_chassis.urdf.xacro`: Robot description

## Controller Architecture

The system uses ros2_controllers framework with:
- **joint_state_broadcaster**: Publishes joint states
- **mecanum_drive_controller**: Handles mecanum wheel kinematics
- Topic remapping: `/cmd_vel` → `/mecanum_drive_controller/reference_unstamped`

## Simulation vs Hardware

- **Simulation**: Uses ros_ign_gazebo (Ignition Gazebo) with ros_ign_bridge
- **Hardware**: USB serial communication with Yahboom expansion board (115200 baud)
- Both modes use same controller configuration and launch structure

## Development Workflow

1. Make changes in `src/` directory
2. Build with `colcon build --symlink-install` 
3. Source workspace: `source install/setup.bash`
4. Test functionality with appropriate launch files
5. Run tests: `colcon test` for affected packages
6. Commit changes following conventional commit format

## Important Notes

- Python packages use setuptools with pytest for testing
- C++ packages use ament_cmake build system
- All packages include standard ROS2 linting tests (flake8, copyright, pep257)
- Launch files support both simulation and hardware modes
- Robot description uses xacro for parameterized URDF generation