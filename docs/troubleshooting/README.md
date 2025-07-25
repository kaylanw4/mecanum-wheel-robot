# Troubleshooting Guide

Common issues and solutions for the mecanum wheel robot project.

## Common Issues

### 1. Controller Not Detected
**Symptoms**: `/dev/input/js0` not found, `jstest` fails
**Solutions**:
- Re-pair PS4 controller using bluetoothctl
- Check Bluetooth service: `sudo systemctl status bluetooth`
- Install ds4drv: `sudo apt install ds4drv`

### 2. USB Serial Connection Issues
**Symptoms**: Yahboom board not detected, no `/dev/ttyUSB0`
**Solutions**:
- Check USB cable connection
- Verify with `dmesg | tail` when plugging in
- Add user to dialout group: `sudo usermod -a -G dialout $USER`

### 3. ROS2 Build Failures
**Symptoms**: `colcon build` fails with missing dependencies
**Solutions**:
- Update rosdep: `rosdep update`
- Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Check package.xml dependencies

## Getting Help

1. Check this troubleshooting guide
2. Review setup documentation
3. Check GitHub issues
4. Create new issue with detailed error description
