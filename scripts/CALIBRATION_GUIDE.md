# Mecanum Robot Velocity Calibration Guide

## Overview

This guide provides step-by-step instructions for calibrating the velocity control of your mecanum wheel robot using the professional calibration tool. The calibration process ensures that when you command the robot to move at 0.5 m/s, it actually moves at 0.5 m/s.

## What Gets Calibrated

The calibration tool tests and calibrates all three degrees of freedom for your mecanum robot:

1. **Linear X (Forward/Backward)** - Forward and reverse motion
2. **Linear Y (Strafe)** - Side-to-side movement 
3. **Angular Z (Rotation)** - Rotation around the vertical axis

## Prerequisites

### Hardware Requirements
- Mecanum wheel robot with working odometry
- PS4 controller (optional, for emergency stop)
- At least 5x5 meter clear, level space for testing
- USB serial connection to robot hardware

### Software Requirements
- ROS2 workspace built and sourced
- Robot hardware driver running
- Working odometry (`/odom` topic)

### Safety Requirements
- Clear testing area free of obstacles
- Emergency stop method ready (joystick or keyboard interrupt)
- Visual monitoring during all tests
- Level ground surface

## Pre-Calibration Setup

### 1. Prepare Testing Environment

Ensure you have adequate space:
```bash
# Minimum recommended testing area: 5m x 5m clear space
# Robot will move up to 4 meters in straight lines
# Additional space needed for strafe and rotation tests
```

### 2. Start Robot System

Launch your robot with the enhanced system:
```bash
# Terminal 1: Start enhanced robot hardware (WITHOUT joystick for calibration)
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_bringup enhanced_robot_bringup.launch.py use_joystick:=false

# Terminal 2: Verify odometry is working
ros2 topic echo /odom --once
```

**Important**: We disable the joystick (`use_joystick:=false`) during calibration to prevent interference with the calibration tool's velocity commands.

### 3. Verify System Status

Check that all required topics are available:
```bash
# Check required topics (enhanced system has velocity smoother)
ros2 topic list | grep -E "(cmd_vel|odom)"

# Expected output:
# /cmd_vel           # Raw commands to robot
# /cmd_vel_nav       # Smoothed commands from Nav2
# /odom              # Odometry from robot

# Verify nodes are running
ros2 node list | grep -E "(yahboom|velocity)"

# Expected output:
# /yahboom_driver
# /velocity_smoother

# Test basic robot movement (commands go directly to robot, bypassing smoother)
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}}'
# Robot should move forward slowly

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
```

## Running the Calibration

### 1. Navigate to Scripts Directory
```bash
cd ~/ros2_ws/scripts
```

### 2. Make Script Executable
```bash
chmod +x velocity_calibration.py
```

### 3. Run Calibration Tool
```bash
# Terminal 3: Run calibration (keep robot terminals running)
python3 velocity_calibration.py
```

### 4. Follow Interactive Prompts

The tool will display:
- Safety warnings and requirements
- Testing overview
- Confirmation prompt

**Type 'y' and press Enter to proceed with calibration.**

## Calibration Process

### Test Sequence

The calibration runs automatically through these phases:

#### Phase 1: Linear X Calibration (Forward/Backward)
- Tests velocities: 0.1, 0.2, 0.3, 0.5, 0.8 m/s
- Each velocity tested in both forward and backward directions
- 5 seconds per test with 2-second stabilization between tests

#### Phase 2: Linear Y Calibration (Strafe Left/Right)  
- Tests velocities: 0.1, 0.2, 0.3, 0.5, 0.8 m/s
- Each velocity tested in both left and right directions
- 5 seconds per test with 2-second stabilization between tests

#### Phase 3: Angular Z Calibration (Rotation)
- Tests velocities: 0.2, 0.5, 0.8, 1.0, 1.5 rad/s
- Each velocity tested in both clockwise and counterclockwise directions
- 5 seconds per test with 2-second stabilization between tests

### What to Expect

During each test you'll see:
```
🧪 Test 1/30: Linear X at 0.1 m/s
⏳ Stabilizing for 2.0s...
📍 Start position: X=0.000m, Y=0.000m, Θ=0.0°
🚀 Executing motion for 5.0s...
📍 End position: X=0.485m, Y=0.003m, Θ=1.2°
📊 Results: Displacement=0.485, Measured Velocity=0.097, Error=-3.0%
```

### Emergency Stop

If needed, stop the calibration immediately:
- **Joystick**: Press Share + Options buttons simultaneously
- **Keyboard**: Press Ctrl+C in the terminal running the script

## Understanding Results

### Real-Time Feedback

During calibration, you'll see:
- Current test progress (e.g., "Test 5/30")
- Robot position before and after each test
- Measured vs commanded velocity
- Error percentage for each test

### Final Analysis

After all tests complete, the tool provides:
- Statistical analysis for each motion type
- Recommended calibration factors
- R² correlation values (should be > 0.95 for good calibration)
- Configuration file updates

Example output:
```
🎯 CALIBRATION SUMMARY
============================================================

Linear X:
  Mean Error: -2.5% (±1.2%)
  Correlation: R² = 0.998
  Scale Factor: 1.025

Linear Y:
  Mean Error: +4.1% (±2.3%)
  Correlation: R² = 0.995
  Scale Factor: 0.961

Angular Z:
  Mean Error: -1.8% (±1.5%)
  Correlation: R² = 0.997
  Scale Factor: 1.018
```

### Generated Files

The calibration tool creates several files in `/home/kaylanw4/ros2_ws/calibration_results/`:

1. **Detailed CSV Data** (`velocity_calibration_data_YYYYMMDD_HHMMSS.csv`)
   - Raw data from all tests
   - Positions, velocities, errors, timestamps

2. **Calibration Report** (`calibration_report_YYYYMMDD_HHMMSS.txt`)
   - Complete analysis summary
   - Configuration recommendations
   - Individual test results

## Applying Calibration Results

### 1. Update Hardware Configuration

Add the recommended scale factors to your `hardware.yaml` file:

```yaml
# In src/robot_bringup/config/hardware.yaml
enhanced_yahboom_driver:
  ros__parameters:
    # ... existing parameters ...
    
    # Add these calibration factors from your results:
    velocity_scale_x: 1.025    # From Linear X calibration
    velocity_scale_y: 0.961    # From Linear Y calibration  
    velocity_scale_z: 1.018    # From Angular Z calibration
```

**Note**: The enhanced system uses the same `hardware.yaml` configuration file, so calibration parameters are applied the same way.

### 2. Rebuild and Test

```bash
# Rebuild the workspace
cd ~/ros2_ws
colcon build --packages-select robot_hardware robot_bringup

# Source the updated workspace
source install/setup.bash

# Restart the enhanced robot system (with joystick re-enabled)
ros2 launch robot_bringup enhanced_robot_bringup.launch.py
```

### 3. Verify Calibration

Test the calibrated robot:
```bash
# Test forward motion at 0.5 m/s
ros2 topic pub --rate 1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# Measure actual distance traveled in 10 seconds
# Should be very close to 5.0 meters

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{}'
```

## Troubleshooting

### Common Issues

#### No Odometry Data
```bash
# Check if odometry topic exists
ros2 topic list | grep odom

# Check odometry data
ros2 topic echo /odom --once

# If no data, restart hardware driver
```

#### Robot Doesn't Move
```bash
# Check cmd_vel is being published
ros2 topic echo /cmd_vel

# Check hardware connection
ls /dev/ttyUSB*

# Verify robot_hardware node is running
ros2 node list | grep yahboom
```

#### Poor Calibration Results (R² < 0.95)
- Ensure level testing surface
- Check for wheel slippage
- Verify encoder functionality
- Re-run calibration with more care to initial positioning

#### Robot Drifts During Straight Line Tests
- This is normal and expected - the calibration accounts for this
- Drifting indicates motor imbalances that calibration will correct
- Excessive drift (>30°) may indicate mechanical issues

### Emergency Procedures

#### If Robot Moves Unexpectedly
1. **Immediate**: Press Ctrl+C or joystick emergency stop
2. **Physical**: Power off robot if necessary
3. **Recovery**: Check all connections and restart system

#### If Calibration Gives Bad Results
1. Verify clear testing space and level ground
2. Ensure robot starts from complete stop
3. Check odometry accuracy manually
4. Re-run calibration focusing on consistent initial conditions

## Quality Assurance

### Good Calibration Indicators
- R² correlation > 0.95 for all motion types
- Error standard deviation < 5%
- Consistent results across velocity ranges
- Robot moves predictably after applying calibration

### Poor Calibration Indicators
- R² correlation < 0.90
- Error standard deviation > 10%
- Inconsistent velocity responses
- Robot still doesn't move at commanded speeds

### When to Re-Calibrate
- After mechanical changes to robot
- If navigation accuracy degrades
- After encoder or motor maintenance
- Seasonally for production robots

## Advanced Options

### Custom Test Velocities

Edit the calibration script to change test velocities:
```python
# In velocity_calibration.py, modify these values:
self.test_velocities = {
    'linear_x': [0.1, 0.2, 0.3, 0.5, 0.8],  # Add/remove velocities
    'linear_y': [0.1, 0.2, 0.3, 0.5, 0.8],  # Customize as needed
    'angular_z': [0.2, 0.5, 0.8, 1.0, 1.5]  # Adjust for your robot
}
```

### Longer Test Duration

For more accurate measurements, increase test duration:
```python
self.test_duration = 10.0  # seconds per test (default: 5.0)
```

### Custom Output Directory

Change where results are saved:
```python
self.output_dir = '/path/to/your/calibration/results'
```

## Support

If you encounter issues:
1. Check this guide's troubleshooting section
2. Verify all prerequisites are met
3. Review the calibration logs for error messages
4. Ensure robot hardware is functioning properly

The calibration tool is designed to be robust and provide clear feedback about any issues encountered.