#!/bin/bash

# Save SLAM Map Script
# File: src/robot_bringup/scripts/save_slam_map.sh
# Usage: ./save_slam_map.sh [map_name]

MAP_NAME=${1:-"my_map"}
MAP_DIR="$HOME/ros2_ws/maps"

# Create maps directory if it doesn't exist
mkdir -p "$MAP_DIR"

echo "📍 Saving SLAM map as: $MAP_NAME"
echo "📁 Map directory: $MAP_DIR"

# Save the map using the map_saver service
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{
  map_topic: 'map',
  map_url: '$MAP_DIR/$MAP_NAME',
  image_format: 'pgm',
  map_mode: 'trinary',
  free_thresh: 0.25,
  occupied_thresh: 0.65
}"

if [ $? -eq 0 ]; then
    echo "✅ Map saved successfully!"
    echo "📄 Files created:"
    echo "   - $MAP_DIR/$MAP_NAME.pgm (map image)"
    echo "   - $MAP_DIR/$MAP_NAME.yaml (map metadata)"
    echo ""
    echo "🗺️  You can now use this map for navigation with:"
    echo "   ros2 launch robot_bringup robot_navigation.launch.py map:=$MAP_DIR/$MAP_NAME.yaml"
else
    echo "❌ Failed to save map. Make sure SLAM is running and a map has been built."
fi