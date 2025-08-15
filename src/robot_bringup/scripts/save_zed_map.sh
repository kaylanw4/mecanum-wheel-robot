#!/bin/bash
# ZED Map Saving Script - Industry Standard Multi-Map Support
# File: src/robot_bringup/scripts/save_zed_map.sh
# Usage: ./save_zed_map.sh [map_name]
# Example: ./save_zed_map.sh office_map

set -e

# Parse command line arguments
CUSTOM_MAP_NAME="$1"
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
MAP_DIR="/tmp/zed_maps"

# Determine map name
if [ -n "$CUSTOM_MAP_NAME" ]; then
    MAP_NAME="$CUSTOM_MAP_NAME"
    echo "🗺️ Using custom map name: $MAP_NAME"
else
    MAP_NAME="zed_map_${TIMESTAMP}"
    echo "🗺️ Using timestamp-based map name: $MAP_NAME"
fi

AREA_MEMORY_PATH="/tmp/zed_area_memory.area"

echo "🗺️  ZED Map Saving Script - Industry Standard"
echo "=============================================="

# Create map directory if it doesn't exist
mkdir -p "${MAP_DIR}"

echo "📁 Map directory: ${MAP_DIR}"
echo "⏰ Timestamp: ${TIMESTAMP}"

# Check if ZED node is running
if ! ros2 node list | grep -q "zed_node"; then
    echo "❌ ERROR: ZED node is not running!"
    echo "   Please launch the mapping system first:"
    echo "   ros2 launch robot_bringup robot_mapping.launch.py"
    exit 1
fi

echo "🎯 ZED node detected - proceeding with map save..."

# Check if map already exists and ask for confirmation
if [ -f "${MAP_DIR}/${MAP_NAME}.area" ]; then
    echo "⚠️  Map '${MAP_NAME}' already exists!"
    echo "   Existing: ${MAP_DIR}/${MAP_NAME}.area"
    echo -n "   Overwrite? (y/N): "
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "❌ Map save cancelled"
        exit 1
    fi
    echo "🔄 Overwriting existing map..."
fi

# Save area memory database (for relocalization)
if [ -f "${AREA_MEMORY_PATH}" ]; then
    cp "${AREA_MEMORY_PATH}" "${MAP_DIR}/${MAP_NAME}.area"
    echo "✅ Area memory saved: ${MAP_DIR}/${MAP_NAME}.area"
else
    echo "⚠️  Warning: Area memory file not found at ${AREA_MEMORY_PATH}"
    # Call the save service to create area memory
    echo "📞 Calling ZED save_area_memory service..."
    if ros2 service call /zed2i/zed_node/save_area_memory zed_interfaces/srv/save_area_memory "{}"; then
        echo "✅ Area memory service called successfully"
        sleep 2
        if [ -f "${AREA_MEMORY_PATH}" ]; then
            cp "${AREA_MEMORY_PATH}" "${MAP_DIR}/${MAP_NAME}.area"
            echo "✅ Area memory saved: ${MAP_DIR}/${MAP_NAME}.area"
        fi
    else
        echo "❌ Failed to call save_area_memory service"
    fi
fi

# Save current pose for map origin reference
echo "📍 Saving current robot pose..."
ros2 topic echo /zed2i/zed_node/pose -1 > "${MAP_DIR}/${MAP_NAME}_pose.yaml"
echo "✅ Robot pose saved: ${MAP_DIR}/${MAP_NAME}_pose.yaml"

# Save map metadata
echo "📋 Creating map metadata..."
cat > "${MAP_DIR}/${MAP_NAME}_info.yaml" << EOF
# ZED Map Information
map_name: ${MAP_NAME}
timestamp: ${TIMESTAMP}
created_by: ZED2i VI-SLAM Phase 4
area_memory_file: ${MAP_NAME}.area
pose_file: ${MAP_NAME}_pose.yaml

# Map parameters
resolution: 0.05  # 5cm resolution
max_range: 15.0   # 15m mapping range
coordinate_frame: map

# Usage instructions
# To load this map:
# 1. Copy ${MAP_NAME}.area to /tmp/zed_area_memory.area
# 2. Launch: ros2 launch robot_bringup robot_mapping.launch.py
# 3. ZED should automatically relocalize using the area memory
EOF

echo "✅ Map metadata saved: ${MAP_DIR}/${MAP_NAME}_info.yaml"

# List saved files
echo ""
echo "🎉 Map saved successfully!"
echo "📂 Saved files:"
ls -la "${MAP_DIR}/${MAP_NAME}"*

echo ""
echo "📖 To load this map for localization:"
if [ -n "$CUSTOM_MAP_NAME" ]; then
    echo "   ros2 launch robot_bringup robot_localization.launch.py map_name:=${MAP_NAME}"
else
    echo "   # Using map name:"
    echo "   ros2 launch robot_bringup robot_localization.launch.py map_name:=${MAP_NAME}"
    echo ""
    echo "   # Or copy manually:"
    echo "   cp ${MAP_DIR}/${MAP_NAME}.area /tmp/zed_area_memory.area"
    echo "   ros2 launch robot_bringup robot_localization.launch.py"
fi
echo ""
echo "📋 Available maps:"
ls -la "${MAP_DIR}"/*.area 2>/dev/null | awk '{print "   " $9}' | sed 's|.*/||' | sed 's|\.area$||'
echo ""
echo "✨ ZED map saving completed!"