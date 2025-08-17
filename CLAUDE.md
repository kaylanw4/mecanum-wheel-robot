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

# Robot with ZED2i camera integration
ros2 launch robot_bringup robot_with_zed2i.launch.py

# ZED2i Visual-Inertial SLAM (VI-SLAM) - Primary Mode
ros2 launch robot_bringup robot_vi_slam.launch.py

# ZED2i VI-SLAM camera only
ros2 launch robot_bringup zed_vi_slam.launch.py

# ZED2i mapping mode (fresh map creation)
ros2 launch robot_bringup robot_mapping.launch.py

# ZED2i localization mode (use existing map)
ros2 launch robot_bringup robot_localization.launch.py map_name:=office_map

# ZED2i navigation mode (autonomous navigation)
ros2 launch robot_bringup robot_navigation.launch.py map_name:=office_map

# Save ZED VI-SLAM map
bash src/robot_bringup/scripts/save_zed_map.sh office_map
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

# Monitor ZED2i camera topics
ros2 topic echo /zed2i/zed_node/rgb/image_rect_color
ros2 topic echo /zed2i/zed_node/depth/depth_registered
ros2 topic echo /zed2i/zed_node/point_cloud/cloud_registered

# Monitor ZED2i VI-SLAM topics
ros2 topic echo /zed2i/zed_node/pose
ros2 topic echo /zed2i/zed_node/pose_with_covariance
ros2 topic echo /zed2i/zed_node/path_map
ros2 topic echo /zed2i/zed_node/path_odom
ros2 topic echo /zed2i/zed_node/mapping/fused_cloud
ros2 topic echo /zed2i/zed_node/pose/status

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
- **zed-ros2-wrapper**: ZED2i camera driver and integration
- **zed-ros2-examples**: ZED camera examples and tutorials

### Hardware Interface
The system uses a USB serial connection to communicate with the Yahboom Robot Expansion Board V3.0 (STM32F103RCT6). The `yahboom_driver.py` integrates the official Yahboom `Rosmaster_Lib` library to:
- Control 4x mecanum wheels with encoders (520 CPR)  
- Read IMU data (accelerometer, gyroscope, magnetometer)
- Monitor battery voltage
- Control RGB lights and buzzer
- Publish odometry and joint states

### ZED2i Camera with Visual-Inertial SLAM
The ZED2i stereo camera provides Visual-Inertial SLAM (VI-SLAM) capabilities:
- High-resolution RGB and depth images (720p at 30fps)
- 3D point cloud data with spatial mapping
- Visual-inertial odometry combining camera and IMU data
- Real-time pose tracking and trajectory estimation
- Loop closure detection for map consistency
- Area memory for relocalization and map saving/loading
- Multi-map support with named map management

### Control Flow (VI-SLAM Mode)
1. **Joy node** reads PS4 controller input → `/joy` topic
2. **Yahboom joystick controller** processes joy input with gear modes → `/cmd_vel` topic  
3. **Yahboom hardware driver** receives cmd_vel → sends to robot via USB serial (wheel odometry disabled)
4. **Robot state publisher** manages transforms from URDF
5. **Joint state publisher** handles wheel joint states for visualization
6. **ZED2i node** provides VI-SLAM → pose estimation, spatial mapping, and visual odometry
7. **ZED2i transforms** publishes map→odom→camera_link with camera tracking as primary odometry
8. **Area memory system** enables map saving/loading and relocalization

### Key Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands
- `/joy` (sensor_msgs/Joy): Joystick input
- `/imu/data_raw` (sensor_msgs/Imu): Robot IMU sensor data
- `/joint_states` (sensor_msgs/JointState): Wheel positions
- `/battery_voltage` (std_msgs/Float32): Battery status
- `/rgb_light` (std_msgs/Int32): RGB light control
- `/buzzer` (std_msgs/Bool): Buzzer control

**ZED2i VI-SLAM Topics:**
- `/zed2i/zed_node/rgb/image_rect_color` (sensor_msgs/Image): RGB camera feed
- `/zed2i/zed_node/depth/depth_registered` (sensor_msgs/Image): Depth data
- `/zed2i/zed_node/point_cloud/cloud_registered` (sensor_msgs/PointCloud2): 3D point cloud
- `/zed2i/zed_node/pose` (geometry_msgs/PoseStamped): ZED pose in map frame
- `/zed2i/zed_node/pose_with_covariance` (geometry_msgs/PoseWithCovarianceStamped): Pose with uncertainty
- `/zed2i/zed_node/path_map` (nav_msgs/Path): Trajectory in map frame
- `/zed2i/zed_node/path_odom` (nav_msgs/Path): Trajectory in odom frame
- `/zed2i/zed_node/mapping/fused_cloud` (sensor_msgs/PointCloud2): Spatial mapping point cloud
- `/zed2i/zed_node/pose/status` (zed_interfaces/PosTrackStatus): Tracking quality status

### Configuration Files
- Hardware parameters: `src/robot_bringup/config/hardware.yaml`
- Joystick mappings: `src/robot_bringup/config/joystick.yaml` 
- Robot URDF: `src/robot_bringup/urdf/yahboomcar_chassis.urdf.xacro`
- Robot with ZED2i URDF: `src/robot_bringup/urdf/yahboomcar_with_zed2i.urdf.xacro`
- Robot VI-SLAM URDF: `src/robot_bringup/urdf/yahboomcar_vi_slam.urdf.xacro`
- RViz config: `src/robot_bringup/config/robot_view.rviz`
- ZED2i RViz config: `src/robot_bringup/config/robot_with_zed2i.rviz`

**ZED2i VI-SLAM Configuration:**
- VI-SLAM config: `src/robot_bringup/config/zed_vi_slam_config.yaml`
- Mapping config: `src/robot_bringup/config/zed_mapping_config.yaml`
- Localization config: `src/robot_bringup/config/zed_localization_config.yaml`
- Navigation config: `src/robot_bringup/config/zed_navigation_config.yaml`

## Important Development Notes

### Hardware Dependencies
- Requires Yahboom `Rosmaster_Lib` Python library for hardware communication
- USB serial connection on `/dev/ttyUSB0` (configurable via launch parameter)
- PS4 controller paired via Bluetooth for joystick control
- ZED2i camera connected via USB 3.0
- ZED SDK installed for camera functionality

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

### Visual-Inertial SLAM and Navigation
The system supports advanced VI-SLAM capabilities:
- Real-time visual-inertial SLAM using ZED2i camera and IMU fusion
- Spatial mapping with 3D point cloud generation and fused mapping
- Loop closure detection for map consistency and drift correction
- Multi-map support with named map management (office_map, warehouse_map, etc.)
- Area memory system for relocalization and map persistence
- Three operational modes: Mapping (fresh), Localization (existing), Navigation (autonomous)
- Professional map management tools with conflict-free operation

### Motor Calibration
Use the motor calibration tool for optimal performance:
```bash
# Run motor calibration (interactive)
python3 src/robot_hardware/robot_hardware/motor_calibration_tool.py

# Test motor response
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```