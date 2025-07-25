# Mecanum Wheel Robot - ROS2 Project

A comprehensive ROS2-based mecanum wheel robot project using NVIDIA Jetson Orin Nano and Yahboom hardware components for autonomous robotics development and education.

## 🤖 Hardware Components

- **Main Controller**: NVIDIA Jetson Orin Nano 8GB Development Kit
- **Expansion Board**: Yahboom Robot Expansion Board V3.0 (STM32F103RCT6)
- **Chassis**: Yahboom Aluminum Alloy Car Chassis with 80mm Mecanum Wheels
- **Motors**: 4x DC motors with 520 CPR encoders
- **Input Device**: PS4 DualShock wireless controller
- **Communication**: USB serial connection (no GPIO wiring required)
- **Power**: 12V battery system with integrated power management

## 💻 Software Stack

- **Operating System**: Ubuntu 22.04.3 LTS (JetPack 6.2.1)
- **ROS Version**: ROS2 Humble Hawksbill (LTS)
- **Simulation**: Gazebo Fortress with ros-gz integration
- **Control Framework**: ros2_controllers with mecanum drive controller
- **Hardware Interface**: Custom STM32 communication protocol
- **Navigation**: Future integration with Nav2 stack

## 📁 Project Structure

```
ros2_ws/
├── src/
│   ├── robot_bringup/           # Launch files and system configuration
│   │   ├── launch/              # ROS2 launch files
│   │   ├── config/              # Parameter configurations
│   │   └── README.md
│   ├── robot_description/       # URDF models and robot description
│   │   ├── urdf/                # Robot URDF and Xacro files
│   │   ├── meshes/              # 3D model files
│   │   ├── config/              # Robot-specific configurations
│   │   └── README.md
│   ├── robot_teleop/           # Teleoperation and joystick control
│   │   ├── launch/              # Teleop launch files
│   │   ├── config/              # Joystick and control configs
│   │   ├── src/                 # Teleop node source code
│   │   └── README.md
│   └── robot_hardware/         # Hardware drivers and interfaces
│       ├── launch/              # Hardware interface launches
│       ├── config/              # Hardware configurations
│       ├── src/                 # Driver source code
│       └── README.md
├── docs/                       # Documentation and tutorials
│   ├── setup/                  # Installation and setup guides
│   ├── tutorials/              # Usage tutorials
│   └── troubleshooting/        # Common issues and solutions
├── scripts/                    # Utility and setup scripts
│   ├── installation/           # Installation automation
│   ├── calibration/            # Robot calibration tools
│   └── testing/                # Testing and validation scripts
└── media/                      # Images, videos, and CAD files
    ├── images/                 # Photos and diagrams
    ├── videos/                 # Demo and tutorial videos
    └── cad/                    # CAD files and technical drawings
```

## 🚀 Quick Start

### Prerequisites

Before starting, ensure you have:
- NVIDIA Jetson Orin Nano 8GB with fresh JetPack 6.2.1 installation
- SSD storage device properly configured
- Internet connection for package installation
- PS4 controller available for pairing
- Yahboom hardware components assembled

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/YOUR_USERNAME/mecanum-wheel-robot.git
   cd mecanum-wheel-robot
   ```

2. **Install ROS2 dependencies**
   ```bash
   # Update package lists
   sudo apt update
   
   # Install rosdep if not already installed
   sudo apt install python3-rosdep
   sudo rosdep init
   rosdep update
   
   # Install project dependencies
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**
   ```bash
   # Build all packages
   colcon build --symlink-install
   
   # Source the workspace
   source install/setup.bash
   
   # Add to bashrc for convenience
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   ```

4. **Configure hardware connections**
   ```bash
   # Add user to dialout group for USB serial access
   sudo usermod -a -G dialout $USER
   
   # Logout and login again, or run:
   newgrp dialout
   
   # Connect Yahboom expansion board via USB
   # Verify connection
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

5. **Pair PS4 controller**
   ```bash
   # Install Bluetooth utilities
   sudo apt install bluez bluez-tools
   
   # Pair controller (hold PlayStation + Share buttons)
   bluetoothctl
   # Follow prompts: power on, scan on, pair, trust, connect
   
   # Test controller
   jstest /dev/input/js0
   ```

### Basic Usage

#### 1. Launch Robot System
```bash
# Terminal 1: Start robot hardware drivers
ros2 launch robot_bringup robot_bringup.launch.py

# Verify topics are published
ros2 topic list
ros2 topic echo /joint_states
```

#### 2. Start Teleoperation
```bash
# Terminal 2: Launch joystick control
ros2 launch robot_teleop joystick_teleop.launch.py

# Alternative: Keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/mecanum_drive_controller/cmd_vel
```

#### 3. Run Simulation (Optional)
```bash
# Terminal 3: Start Gazebo simulation
ros2 launch robot_description gazebo_sim.launch.py

# In another terminal: Control simulated robot
ros2 launch robot_teleop joystick_teleop.launch.py use_sim_time:=true
```

#### 4. Monitor System
```bash
# Check system status
ros2 node list
ros2 topic list
ros2 service list

# Monitor transforms
ros2 run rqt_tf_tree rqt_tf_tree

# Visualize in RViz2
ros2 launch robot_description display.launch.py
```

## 🛠️ Development Status

### ✅ Completed Features
- [x] Initial project setup and structure
- [x] Hardware integration documentation
- [x] PS4 controller pairing and configuration
- [x] USB serial communication setup
- [x] ROS2 Humble installation and configuration
- [x] Basic package structure and dependencies
- [x] Development workflow and version control

### 🚧 In Progress
- [ ] Robot URDF description and visualization
- [ ] Yahboom hardware driver implementation
- [ ] Mecanum drive controller configuration
- [ ] Teleoperation system development
- [ ] Gazebo simulation environment

### 🔮 Future Features
- [ ] Autonomous navigation with Nav2
- [ ] SLAM mapping and localization
- [ ] Depth camera integration (ZED/RealSense)
- [ ] Computer vision capabilities
- [ ] Machine learning integration
- [ ] Web-based monitoring dashboard

## 📚 Documentation

### Setup and Installation
- [Complete Setup Guide](docs/setup/README.md) - Full installation walkthrough
- [Hardware Configuration](docs/setup/hardware_setup.md) - Physical assembly guide
- [Software Installation](docs/setup/software_setup.md) - ROS2 and dependencies
- [Controller Setup](docs/setup/controller_setup.md) - PS4 controller pairing

### Development
- [Development Workflow](docs/development/workflow.md) - Git workflow and development practices
- [Package Documentation](docs/development/packages.md) - Individual package details
- [API Reference](docs/development/api.md) - Code documentation
- [Testing Guide](docs/development/testing.md) - Testing procedures

### Usage and Tutorials
- [Basic Operation](docs/tutorials/basic_usage.md) - Getting started tutorial
- [Teleoperation](docs/tutorials/teleop.md) - Manual control guide
- [Simulation](docs/tutorials/simulation.md) - Gazebo simulation usage
- [Calibration](docs/tutorials/calibration.md) - Robot calibration procedures

### Troubleshooting
- [Common Issues](docs/troubleshooting/README.md) - Frequently encountered problems
- [Hardware Troubleshooting](docs/troubleshooting/hardware.md) - Hardware-related issues
- [Software Troubleshooting](docs/troubleshooting/software.md) - Software and ROS2 issues

## 🔧 Hardware Configuration

### Power Requirements
- **Input Voltage**: 12V DC (11-13V range)
- **Power Consumption**: ~30W typical, ~50W peak
- **Battery**: Recommended 12V 5000mAh+ LiPo battery
- **Expansion Board**: Provides 5V output for Jetson (optional)

### Communication Interfaces
- **Primary**: USB serial (115200 baud)
- **Backup**: UART pins available if needed
- **Wireless**: PS4 controller via Bluetooth
- **Future**: Wi-Fi for remote monitoring

### Motor Specifications
- **Type**: DC geared motors with quadrature encoders
- **Encoders**: 520 CPR (2080 after quadrature decoding)
- **Wheel Diameter**: 80mm mecanum wheels
- **Max Speed**: ~1.5 m/s linear, ~3 rad/s angular

## 🔄 Development Workflow

### Git Workflow
```bash
# Create feature branch for new work
git checkout develop
git pull origin develop
git checkout -b feature/your-feature-name

# Make your changes and test thoroughly
git add .
git commit -m "feat: add new teleop functionality

- Implement advanced joystick control modes
- Add safety features and emergency stop
- Update configuration files and documentation"

# Push changes
git push origin feature/your-feature-name

# Merge back to develop when ready
git checkout develop
git merge feature/your-feature-name
```

### Development Standards
- Follow ROS2 naming conventions
- Include comprehensive comments
- Test functionality thoroughly
- Update documentation for changes
- Use descriptive commit messages

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### License Summary
- ✅ Commercial use allowed
- ✅ Modification allowed
- ✅ Distribution allowed
- ✅ Private use allowed
- ❌ No warranty provided
- ❌ No liability assumed

## 🙏 Acknowledgments

### Hardware Partners
- **Yahboom Technology** - For excellent robotics hardware components and documentation
- **NVIDIA** - For the powerful Jetson platform and comprehensive developer tools

### Software Communities
- **ROS2 Community** - For the incredible robotics framework and continuous support
- **Open Source Robotics Foundation** - For Gazebo simulator and related tools
- **Ubuntu/Canonical** - For the stable and reliable operating system foundation

### Educational Resources
- ROS2 tutorials and documentation
- Gazebo simulation tutorials
- NVIDIA Jetson developer resources
- Community forums and Stack Overflow contributors

## 📞 Support and Documentation

### Getting Help
1. **Check Documentation**: Start with the comprehensive docs in the `/docs` folder
2. **Review Troubleshooting**: Check the troubleshooting guide for common issues
3. **Examine Logs**: Use ROS2 logging to debug issues (`ros2 topic echo`, `rqt_console`)
4. **Community Resources**: Consult ROS2 community forums and documentation

### Issue Debugging
When encountering issues, gather this information:
- Hardware configuration details
- Software versions (ROS2, Ubuntu, JetPack)
- Complete error messages and logs
- Steps that led to the problem
- Expected vs actual behavior

### Useful Debugging Commands
```bash
# Check ROS2 system status
ros2 doctor
ros2 node list
ros2 topic list

# Monitor system resources
htop
nvidia-smi  # GPU usage on Jetson

# View logs
ros2 log info
journalctl -f  # System logs
```

---
