# Complete ZED VI-SLAM Mapping Process Guide - Industry Standard

## Overview
This guide walks you through the industry-standard multi-map workflow using ZED2i VI-SLAM. Features professional map management with named maps, conflict-free mapping, and easy map selection for localization.

## Prerequisites
- Robot must be on level ground with good lighting
- ZED2i camera connected and calibrated
- PS4 controller paired (for manual driving)
- Workspace built with industry-standard configuration

## Industry-Standard Architecture

### **Three Operational Modes:**
1. **MAPPING MODE**: Create fresh maps (no area memory conflicts)
2. **LOCALIZATION MODE**: Use existing maps for relocalization
3. **NAVIGATION MODE**: Full autonomous navigation with nav2

### **Multi-Map Management:**
- **Named maps**: Create maps with custom names (e.g., "office_map", "warehouse_map")  
- **Map selection**: Choose specific maps for localization
- **Conflict prevention**: Fresh SLAM graphs prevent vertex errors
- **Professional tools**: Map management utilities included

---

## Step 1: Launch the Mapping System (MAPPING MODE)

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
- Starts ZED2i with **FRESH MAPPING** mode (no area memory conflicts)
- Enables VI-SLAM with loop closure detection
- Uses `zed_mapping_config.yaml` (area_memory: false)
- Launches robot hardware drivers (wheels disabled for odometry)
- Starts joystick control for manual driving
- Opens RViz for real-time visualization

**Key Improvement**: No more "vertex already registered" errors!

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

## Step 5: Save the Map (Multi-Map Support)

### After 5-10 minutes of mapping:

**Stop robot movement** (release all joystick controls)

### Terminal 2: Save Named Map
```bash
# Save with custom name (RECOMMENDED)
./src/robot_bringup/scripts/save_zed_map.sh office_map

# Or save with timestamp (default)
./src/robot_bringup/scripts/save_zed_map.sh
```

**Expected output:**
```
🗺️ Using custom map name: office_map
🗺️  ZED Map Saving Script - Industry Standard
==============================================
📁 Map directory: /tmp/zed_maps
🎯 ZED node detected - proceeding with map save...
✅ Area memory saved: /tmp/zed_maps/office_map.area
📍 Saving current robot pose...
✅ Robot pose saved: /tmp/zed_maps/office_map_pose.yaml
📋 Creating map metadata...
✅ Map metadata saved: /tmp/zed_maps/office_map_info.yaml
🎉 Map saved successfully!

📖 To load this map for localization:
   ros2 launch robot_bringup robot_localization.launch.py map_name:=office_map
```

---

## Step 6: Map Management

### Check Your Map Collection:
```bash
# List all saved maps
./src/robot_bringup/scripts/manage_zed_maps.sh list

# Get detailed info about specific map
./src/robot_bringup/scripts/manage_zed_maps.sh info office_map

# See what maps you have
./src/robot_bringup/scripts/manage_zed_maps.sh
```

**Example output:**
```
📋 Available ZED Maps
====================

Map Name             Created         Size       Files
--------             -------         ----       -----
office_map           2025-08-14 15:30  12M        3 files
warehouse_map        2025-08-14 14:15   8.5M       3 files  
lab_map              2025-08-14 13:45   5.2M       3 files

Total maps: 3
🎯 Active area memory: 12M (modified: 2025-08-14 15:32)
```

---

## Step 7: Test Map Loading (LOCALIZATION MODE)

### Terminal 1: Stop Current Mapping System
```bash
# In Terminal 1, press Ctrl+C to stop the mapping system
```

### Load Specific Map for Localization:
```bash
# Load your specific map by name
ros2 launch robot_bringup robot_localization.launch.py map_name:=office_map use_rviz:=true

# Or load different maps
ros2 launch robot_bringup robot_localization.launch.py map_name:=warehouse_map use_rviz:=true
```

**Expected result:**
- ZED automatically relocalizes using selected map
- Robot pose aligns with mapped environment  
- No "vertex already registered" errors
- Clean localization without conflicts

---

## Troubleshooting

### ✅ **Vertex Conflict Fixed!**
The "vertex already registered" error is **completely resolved** with the new industry-standard architecture:
- **MAPPING mode**: Uses fresh SLAM graph (no area memory)
- **LOCALIZATION mode**: Uses existing area memory safely
- **No more crashes** during mapping sessions

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

### Map Management Issues:
```bash
# Clear area memory for fresh mapping
./src/robot_bringup/scripts/manage_zed_maps.sh clear

# Delete problematic maps
./src/robot_bringup/scripts/manage_zed_maps.sh delete old_map

# Check available maps
./src/robot_bringup/scripts/manage_zed_maps.sh list
```

---

## Complete Session Timeline

| Time | Action | Terminal | Details |
|------|--------|----------|---------|
| 0:00 | Launch mapping | Terminal 1 | `ros2 launch robot_mapping.launch.py` (FRESH mode) |
| 0:30 | Verify status | Terminal 2 | Check topics and tracking |
| 1:00 | Start mapping | Controller | Slow movement, figure-8 pattern |
| 4:00 | Loop closure test | Controller | Return to start position |
| 6:00 | Extended mapping | Controller | Explore larger area |
| 10:00 | Save named map | Terminal 2 | `save_zed_map.sh office_map` |
| 10:30 | Test localization | Terminal 1 | `robot_localization.launch.py map_name:=office_map` |
| 11:00 | Map management | Terminal 2 | `manage_zed_maps.sh list` |

**Total time:** 10-15 minutes for complete mapping + testing

**New Benefits:**
- ✅ **No vertex conflicts** - mapping always works
- 🗂️ **Named maps** - professional organization
- 🔄 **Easy switching** - select any map for localization
- 🛠️ **Management tools** - professional map operations

## Related Files

### Launch Files - Industry Standard
- `robot_mapping.launch.py` - **MAPPING MODE**: Fresh map creation (no conflicts)
- `robot_localization.launch.py` - **LOCALIZATION MODE**: Use existing maps
- `robot_navigation.launch.py` - **NAVIGATION MODE**: Full nav2 integration

### Configuration Files - Mode Specific  
- `zed_mapping_config.yaml` - Mapping mode (area_memory: false)
- `zed_localization_config.yaml` - Localization mode (area_memory: true)  
- `zed_navigation_config.yaml` - Navigation mode (nav2 optimized)
- `mapping_view.rviz` - RViz configuration for mapping visualization

### Scripts - Multi-Map Support
- `save_zed_map.sh [map_name]` - Save named maps
- `manage_zed_maps.sh [command]` - Professional map management
- `hardware.yaml` - Robot hardware configuration

### Map Storage - Organized
- `/tmp/zed_maps/` - Named map storage (office_map.area, warehouse_map.area)
- `/tmp/zed_area_memory.area` - Active area memory file

## Implementation Status - Industry Standard
- ✅ **MAPPING MODE**: Fresh map creation without conflicts
- ✅ **LOCALIZATION MODE**: Multi-map selection and loading  
- ✅ **MAP MANAGEMENT**: Professional tools and utilities
- ✅ **CONFLICT RESOLUTION**: "Vertex already registered" error fixed
- 🔄 **NAVIGATION MODE**: Nav2 integration (ready for implementation)
- 🔄 **PERFORMANCE OPTIMIZATION**: Advanced features (future)

## Quick Reference Commands

### Create Maps:
```bash
# Named map (recommended)
./src/robot_bringup/scripts/save_zed_map.sh office_map

# Timestamp map  
./src/robot_bringup/scripts/save_zed_map.sh
```

### Use Maps:
```bash  
# Fresh mapping (no conflicts)
ros2 launch robot_bringup robot_mapping.launch.py

# Load specific map
ros2 launch robot_bringup robot_localization.launch.py map_name:=office_map

# Future navigation
ros2 launch robot_bringup robot_navigation.launch.py map_name:=office_map
```

### Manage Maps:
```bash
# List all maps
./src/robot_bringup/scripts/manage_zed_maps.sh list

# Map information  
./src/robot_bringup/scripts/manage_zed_maps.sh info office_map

# Delete map
./src/robot_bringup/scripts/manage_zed_maps.sh delete old_map
```

**🎉 Your mapping system is now industry-standard with conflict-free operation and professional multi-map management!**

---

### Example Usage:
```bash
# Start fresh mapping
zmap

# Save named map  
zsave office_map

# List all maps
zlist

# Load specific map
zload office_map

# Get map info
zinfo office_map

# Build and source workspace
zbuild

# Monitor topics
ztopics
zstatus

# Emergency stop
zstop
```

These aliases will significantly speed up your development workflow with the ZED mapping system!