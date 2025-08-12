# Complete ZED VI-SLAM Mapping Process Guide

## Overview
This guide walks you through building a complete 3D map using ZED2i VI-SLAM, from launch to map saving.

## Prerequisites
- Robot must be on level ground with good lighting
- ZED2i camera connected and calibrated
- PS4 controller paired (for manual driving)
- Workspace built with Phase 4 files

---

## Step 1: Launch the Mapping System

### Terminal 1: Start the Mapping System
```bash
# Navigate to workspace
cd /home/kaylanw4/ros2_ws

# Source the workspace
source install/setup.bash

# Launch mapping mode with joystick control
ros2 launch robot_bringup robot_mapping.launch.py use_rviz:=true use_joystick:=true
```

**What this does:**
- Starts ZED2i with spatial mapping enabled
- Enables VI-SLAM with loop closure detection
- Launches robot hardware drivers (wheels disabled for odometry)
- Starts joystick control for manual driving
- Opens RViz for real-time visualization

**Wait for:** "Camera successfully opened" message and RViz to display robot model

---

## Step 2: Verify System Status

### Terminal 2: Check System Health
```bash
# New terminal
cd /home/kaylanw4/ros2_ws
source install/setup.bash

# Check ZED is publishing mapping data
ros2 topic list | grep mapping
# Should see: /zed2i/zed_node/mapping/fused_cloud

# Check tracking status
ros2 topic echo /zed2i/zed_node/pose/status --once
# Should show: odometry_status: 0 (tracking active)

# Verify point cloud data
ros2 topic hz /zed2i/zed_node/point_cloud/cloud_registered
# Should show ~10 Hz
```

**Expected output:**
- Mapping topics active
- Tracking status = 0 (good tracking)
- Point cloud publishing at 10Hz
- No error messages

---

## Step 3: Manual Robot Movement for Mapping

### Phase A: Initial Environment Scan (2-3 minutes)
**Using PS4 Controller:**
- **L1** (deadman switch) + **Left stick**: Move robot slowly forward (0.2 m/s)
- **Rotate**: Use right stick to turn robot left/right while moving
- **Pattern**: Drive in a figure-8 or square pattern to capture surroundings

**Movement Goals:**
- Move slowly (< 0.3 m/s) for best mapping quality  
- Rotate frequently to capture different angles
- Cover 3-4 meter area around starting position
- Avoid rapid movements or quick turns

**Monitor in RViz:**
- Point cloud should build up environment geometry
- Path trail shows robot trajectory in green
- No gaps or jumps in the trajectory

### Phase B: Loop Closure Test (1-2 minutes)  
**Return to Starting Area:**
- Drive robot back to initial starting position
- Approach from different angle than original path
- **Watch for:** Loop closure detection (pose correction)

**Signs of successful loop closure:**
- Slight jump/adjustment in robot pose when revisiting known area
- Path becomes more consistent
- Point cloud alignment improves

### Phase C: Extended Mapping (3-5 minutes)
**Explore New Areas:**
- Extend mapping to cover larger area (5-10 meters)
- Include different room features (walls, furniture, doorways)
- Maintain visual overlap between mapped areas

---

## Step 4: Monitor Mapping Progress

### Keep Terminal 2 Active for Monitoring:
```bash
# Monitor mapping progress
ros2 topic echo /zed2i/zed_node/path_map | grep -A5 "poses:"
# Shows number of poses in trajectory

# Check area memory status  
ros2 topic echo /zed2i/zed_node/pose/status --once
# spatial_memory_status should be good (0-1)

# Monitor point cloud size
ros2 topic echo /zed2i/zed_node/mapping/fused_cloud | grep -A3 "height:\|width:"
# Should show growing point cloud data
```

**Quality indicators:**
- Smooth trajectory path (no jumps)
- Growing fused point cloud
- Stable tracking status
- Successful loop closures

---

## Step 5: Save the Map

### After 5-10 minutes of mapping:

**Stop robot movement** (release all joystick controls)

### Terminal 2: Save the Map
```bash
# Run the map saving script
/home/kaylanw4/ros2_ws/src/robot_bringup/scripts/save_zed_map.sh
```

**Expected output:**
```
🗺️  ZED Map Saving Script - Phase 4
======================================
📁 Map directory: /tmp/zed_maps
⏰ Timestamp: 2025-08-11_23-45-30
🎯 ZED node detected - proceeding with map save...
✅ Area memory saved: /tmp/zed_maps/zed_map_2025-08-11_23-45-30.area
📍 Saving current robot pose...
✅ Robot pose saved: /tmp/zed_maps/zed_map_2025-08-11_23-45-30_pose.yaml
📋 Creating map metadata...
✅ Map metadata saved: /tmp/zed_maps/zed_map_2025-08-11_23-45-30_info.yaml
🎉 Map saved successfully!
```

---

## Step 6: Verify Map Quality

### Terminal 2: Check Saved Files
```bash
# List saved map files
ls -la /tmp/zed_maps/
# Should show .area, _pose.yaml, and _info.yaml files

# Check map metadata
cat /tmp/zed_maps/zed_map_*_info.yaml
# Shows map parameters and usage instructions
```

---

## Step 7: Test Map Loading (Optional)

### Terminal 1: Stop Current System
```bash
# In Terminal 1, press Ctrl+C to stop the mapping system
```

### Load the Saved Map:
```bash
# Copy area memory to expected location
cp /tmp/zed_maps/zed_map_*_*.area /tmp/zed_area_memory.area

# Restart mapping system
ros2 launch robot_bringup robot_mapping.launch.py use_rviz:=true use_joystick:=false
```

**Expected result:**
- ZED should automatically relocalize using saved area memory
- Robot pose should align with previously mapped environment
- Point cloud should match saved map data

---

## Troubleshooting

### If Tracking Fails:
```bash
# Reset tracking
ros2 service call /zed2i/zed_node/reset_pos_tracking std_srvs/srv/Empty
```

### If Mapping Quality is Poor:
- Move robot slower (< 0.2 m/s)
- Ensure good lighting
- Check for visual features (avoid blank walls)
- Verify camera is clean and unobstructed

### If Loop Closure Fails:
- Return to starting area more precisely
- Approach from same viewing angle
- Ensure sufficient visual overlap
- Check tracking status remains good

---

## Complete Session Timeline

| Time | Action | Terminal | Details |
|------|--------|----------|---------|
| 0:00 | Launch mapping | Terminal 1 | `ros2 launch robot_mapping.launch.py` |
| 0:30 | Verify status | Terminal 2 | Check topics and tracking |
| 1:00 | Start mapping | Controller | Slow movement, figure-8 pattern |
| 4:00 | Loop closure test | Controller | Return to start position |
| 6:00 | Extended mapping | Controller | Explore larger area |
| 10:00 | Save map | Terminal 2 | `save_zed_map.sh` |
| 11:00 | Verify files | Terminal 2 | Check `/tmp/zed_maps/` |

**Total time:** 10-15 minutes for a complete mapping session

This process creates a high-quality 3D map with loop closure detection and relocalization capabilities for autonomous navigation.

## Related Files

### Launch Files
- `robot_mapping.launch.py` - Main mapping launch file
- `robot_vi_slam.launch.py` - Basic VI-SLAM without mapping

### Configuration Files  
- `zed_vi_slam_config.yaml` - ZED VI-SLAM parameters
- `mapping_view.rviz` - RViz configuration for mapping visualization

### Scripts
- `save_zed_map.sh` - Map saving utility
- `hardware.yaml` - Robot hardware configuration

### Map Storage
- `/tmp/zed_maps/` - Default map storage location
- `/tmp/zed_area_memory.area` - Active area memory file

## Phase Implementation Status
- ✅ Phase 1: URDF and TF Setup
- ✅ Phase 2: ZED Positional Tracking  
- ✅ Phase 3: Primary Odometry Source
- ✅ Phase 4: Spatial Mapping and Loop Closure
- 🔄 Phase 5: Nav2 Integration (Next)
- 🔄 Phase 6: Performance Optimization (Future)