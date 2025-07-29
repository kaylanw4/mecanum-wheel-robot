# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Building and Installing
```bash
# Build all packages in the workspace
colcon build --symlink-install

# Build specific package
colcon build --packages-select robot_hardware

# Source the workspace (required after building)
source install/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Testing and Linting
```bash
# Run Python linting (packages use ament_flake8, ament_pep257)
colcon test --packages-select robot_hardware
colcon test-result --verbose

# Test specific functionality
python3 -m pytest src/robot_hardware/test/
```

### Running the Robot System
```bash
# Main robot bringup (hardware + visualization)
ros2 launch robot_bringup robot_bringup.launch.py

# Hardware only (no joystick)
ros2 launch robot_bringup robot_bringup.launch.py use_joystick:=false

# With RViz visualization
ros2 launch robot_bringup robot_bringup.launch.py use_rviz:=true

# Joystick control standalone
ros2 launch robot_bringup joystick.launch.py
```

### Debugging and Monitoring
```bash
# Check hardware connection
ls /dev/tty{USB,ACM}*

# Monitor robot topics
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /imu/data_raw
ros2 topic echo /joint_states

# Test joystick input
ros2 topic echo /joy

# Check system status
ros2 doctor
ros2 node list
```

## Architecture Overview

### Package Structure
- **robot_bringup**: Main launch files, URDF models, configurations
- **robot_hardware**: Yahboom hardware driver using official Rosmaster_Lib
- **robot_teleop**: Enhanced joystick controller with gear modes and RGB/buzzer control  
- **robot_description**: Robot URDF descriptions and meshes
- **mecanum_robot_sim**: Simulation components

### Hardware Interface
The system uses a USB serial connection to communicate with the Yahboom Robot Expansion Board V3.0 (STM32F103RCT6). The `yahboom_driver.py` integrates the official Yahboom `Rosmaster_Lib` library to:
- Control 4x mecanum wheels with encoders (520 CPR)  
- Read IMU data (accelerometer, gyroscope, magnetometer)
- Monitor battery voltage
- Control RGB lights and buzzer
- Publish odometry and joint states

### Control Flow
1. **Joy node** reads PS4 controller input → `/joy` topic
2. **Yahboom joystick controller** processes joy input with gear modes → `/cmd_vel` topic  
3. **Yahboom hardware driver** receives cmd_vel → sends to robot via USB serial
4. **Robot state publisher** manages transforms from URDF
5. **Joint state publisher** handles wheel joint states for visualization

### Key Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/joy` (sensor_msgs/Joy): Joystick input
- `/imu/data_raw` (sensor_msgs/Imu): IMU sensor data
- `/joint_states` (sensor_msgs/JointState): Wheel positions
- `/battery_voltage` (std_msgs/Float32): Battery status
- `/rgb_light` (std_msgs/Int32): RGB light control
- `/buzzer` (std_msgs/Bool): Buzzer control

### Configuration Files
- Hardware parameters: `src/robot_bringup/config/hardware.yaml`
- Joystick mappings: `src/robot_bringup/config/joystick.yaml` 
- Robot URDF: `src/robot_bringup/urdf/yahboomcar_chassis.urdf.xacro`
- RViz config: `src/robot_bringup/config/robot_view.rviz`

## Important Development Notes

### Hardware Dependencies
- Requires Yahboom `Rosmaster_Lib` Python library for hardware communication
- USB serial connection on `/dev/ttyUSB0` (configurable via launch parameter)
- PS4 controller paired via Bluetooth for joystick control

### Joystick Controls (PS4 Controller)
- **L1**: Deadman switch (hold to enable movement)
- **Left stick**: Forward/backward and strafe control
- **Right stick X**: Rotation
- **R1**: Turbo mode (increases speed)
- **Triangle**: Cycle linear gear ratios
- **Square**: Cycle angular gear ratios  
- **Circle**: Cycle RGB light patterns
- **X**: Toggle buzzer on/off

### Serial Port Configuration
The robot communicates via USB serial. Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

### Python Package Structure
Python packages (`robot_hardware`, `robot_teleop`) use standard ROS2 Python structure with:
- `setup.py` for package configuration and entry points
- `package.xml` for ROS2 dependencies
- Source code in package-named subdirectories
- Test files in `test/` directories