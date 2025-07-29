# ROS2 Mecanum Wheel Robot

A ROS2-based mecanum wheel robot using NVIDIA Jetson Orin Nano and Yahboom hardware components.

## Hardware

- **Controller**: NVIDIA Jetson Orin Nano 8GB
- **Expansion Board**: Yahboom Robot Expansion Board V3.0 (STM32F103RCT6)
- **Motors**: 4x mecanum wheels with 520 CPR encoders
- **Input**: PS4 controller via Bluetooth
- **Communication**: USB serial connection
- **Power**: 12V battery system

## Software Stack

- **OS**: Ubuntu 22.04 (JetPack 6.2.1)
- **ROS2**: Humble Hawksbill
- **Hardware Interface**: Yahboom Rosmaster_Lib integration

## Package Structure

```
src/
├── robot_bringup/      # Launch files, URDF models, configurations
├── robot_hardware/     # Yahboom hardware driver
├── robot_teleop/       # Enhanced joystick controller with gear modes
├── robot_description/  # Robot URDF descriptions and meshes
└── mecanum_robot_sim/  # Simulation components
```

## Quick Start

### Setup
```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install
source install/setup.bash

# Configure serial access
sudo usermod -a -G dialout $USER
```

### Usage
```bash
# Launch robot system (hardware + visualization)
ros2 launch robot_bringup robot_bringup.launch.py

# Launch with joystick control
ros2 launch robot_bringup robot_bringup.launch.py use_joystick:=true

# Launch with RViz
ros2 launch robot_bringup robot_bringup.launch.py use_rviz:=true
```

### PS4 Controller
- **L1**: Hold to enable movement (deadman switch)
- **Left stick**: Forward/backward and strafe
- **Right stick X**: Rotation
- **R1**: Turbo mode
- **Triangle/Square**: Cycle gear ratios
- **Circle**: RGB light patterns
- **X**: Toggle buzzer

## Current Status

**✅ Implemented**
- Yahboom hardware driver with Rosmaster_Lib integration
- Enhanced joystick controller with gear modes and RGB/buzzer control
- URDF robot description and visualization
- Complete launch system with configurable options
- IMU data, battery monitoring, and odometry publishing

**🔧 Architecture**
- USB serial communication (115200 baud) at `/dev/ttyUSB0`
- 4x mecanum wheels with 520 CPR encoders
- IMU (accelerometer, gyroscope, magnetometer)
- RGB lights and buzzer control via topics

**📋 Key Topics**
- `/cmd_vel` - Velocity commands
- `/imu/data_raw` - IMU sensor data  
- `/joint_states` - Wheel positions
- `/battery_voltage` - Battery status
- `/rgb_light` - RGB light control
- `/buzzer` - Buzzer control

## Development

### Build and Test
```bash
# Build workspace
colcon build --symlink-install

# Run tests
colcon test --packages-select robot_hardware
colcon test-result --verbose

# Check system
ros2 doctor
ros2 topic list
ros2 topic echo /cmd_vel
```

### Debugging
```bash
# Check hardware connection
ls /dev/tty{USB,ACM}*

# Monitor topics
ros2 topic echo /imu/data_raw
ros2 topic echo /joint_states

# Test joystick
ros2 topic echo /joy
```
