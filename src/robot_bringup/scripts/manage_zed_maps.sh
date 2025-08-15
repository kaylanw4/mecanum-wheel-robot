#!/bin/bash
# ZED Map Management Utility - Industry Standard
# File: src/robot_bringup/scripts/manage_zed_maps.sh
# Usage: ./manage_zed_maps.sh [list|delete|info] [map_name]

set -e

MAP_DIR="/tmp/zed_maps"
ACTIVE_AREA_MEMORY="/tmp/zed_area_memory.area"

show_usage() {
    echo "🗺️  ZED Map Management Utility"
    echo "=============================="
    echo ""
    echo "Usage: $0 [command] [map_name]"
    echo ""
    echo "Commands:"
    echo "  list                    - List all saved maps"
    echo "  info <map_name>         - Show map information"
    echo "  delete <map_name>       - Delete a saved map"
    echo "  load <map_name>         - Load map as active area memory"
    echo "  backup                  - Backup current area memory"
    echo "  clear                   - Clear active area memory"
    echo ""
    echo "Examples:"
    echo "  $0 list"
    echo "  $0 info office_map"
    echo "  $0 delete old_map"
    echo "  $0 load warehouse_map"
    echo ""
}

list_maps() {
    echo "📋 Available ZED Maps"
    echo "===================="
    
    if [ ! -d "$MAP_DIR" ] || [ -z "$(ls -A "$MAP_DIR"/*.area 2>/dev/null)" ]; then
        echo "No maps found in $MAP_DIR"
        return
    fi
    
    echo ""
    printf "%-20s %-15s %-10s %s\n" "Map Name" "Created" "Size" "Files"
    printf "%-20s %-15s %-10s %s\n" "--------" "-------" "----" "-----"
    
    for area_file in "$MAP_DIR"/*.area; do
        if [ -f "$area_file" ]; then
            # Extract map name from filename
            map_name=$(basename "$area_file" .area)
            
            # Get file info
            file_date=$(date -r "$area_file" "+%Y-%m-%d %H:%M" 2>/dev/null || echo "Unknown")
            file_size=$(du -h "$area_file" 2>/dev/null | cut -f1 || echo "Unknown")
            
            # Count related files
            related_files=$(find "$MAP_DIR" -name "${map_name}*" | wc -l)
            
            printf "%-20s %-15s %-10s %d files\n" "$map_name" "$file_date" "$file_size" "$related_files"
        fi
    done
    
    echo ""
    echo "Total maps: $(ls -1 "$MAP_DIR"/*.area 2>/dev/null | wc -l)"
    
    # Show active area memory status
    echo ""
    if [ -f "$ACTIVE_AREA_MEMORY" ]; then
        active_size=$(du -h "$ACTIVE_AREA_MEMORY" 2>/dev/null | cut -f1)
        active_date=$(date -r "$ACTIVE_AREA_MEMORY" "+%Y-%m-%d %H:%M" 2>/dev/null)
        echo "🎯 Active area memory: $active_size (modified: $active_date)"
    else
        echo "ℹ️  No active area memory found"
    fi
}

show_map_info() {
    local map_name="$1"
    if [ -z "$map_name" ]; then
        echo "❌ Error: Map name required for info command"
        show_usage
        exit 1
    fi
    
    local area_file="$MAP_DIR/${map_name}.area"
    local info_file="$MAP_DIR/${map_name}_info.yaml"
    local pose_file="$MAP_DIR/${map_name}_pose.yaml"
    
    echo "🗺️  Map Information: $map_name"
    echo "================================"
    
    if [ ! -f "$area_file" ]; then
        echo "❌ Map not found: $area_file"
        exit 1
    fi
    
    # Show file information
    echo ""
    echo "📁 Files:"
    if [ -f "$area_file" ]; then
        area_size=$(du -h "$area_file" | cut -f1)
        area_date=$(date -r "$area_file" "+%Y-%m-%d %H:%M:%S")
        echo "   Area Memory: $area_file ($area_size, $area_date)"
    fi
    
    if [ -f "$info_file" ]; then
        echo "   Metadata:    $info_file"
    fi
    
    if [ -f "$pose_file" ]; then
        echo "   Pose Data:   $pose_file"
    fi
    
    # Show metadata if available
    if [ -f "$info_file" ]; then
        echo ""
        echo "📋 Metadata:"
        cat "$info_file" | sed 's/^/   /'
    fi
    
    echo ""
    echo "🚀 To load this map:"
    echo "   ros2 launch robot_bringup robot_localization.launch.py map_name:=$map_name"
}

delete_map() {
    local map_name="$1"
    if [ -z "$map_name" ]; then
        echo "❌ Error: Map name required for delete command"
        show_usage
        exit 1
    fi
    
    local area_file="$MAP_DIR/${map_name}.area"
    
    if [ ! -f "$area_file" ]; then
        echo "❌ Map not found: $area_file"
        exit 1
    fi
    
    echo "⚠️  Delete map '$map_name'?"
    echo "   This will permanently remove:"
    find "$MAP_DIR" -name "${map_name}*" | sed 's/^/   - /'
    echo ""
    echo -n "   Continue? (y/N): "
    read -r response
    
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "❌ Delete cancelled"
        exit 1
    fi
    
    # Delete all related files
    find "$MAP_DIR" -name "${map_name}*" -delete
    
    echo "✅ Map '$map_name' deleted successfully"
}

load_map() {
    local map_name="$1"
    if [ -z "$map_name" ]; then
        echo "❌ Error: Map name required for load command"
        show_usage
        exit 1
    fi
    
    local area_file="$MAP_DIR/${map_name}.area"
    
    if [ ! -f "$area_file" ]; then
        echo "❌ Map not found: $area_file"
        exit 1
    fi
    
    # Backup current area memory if it exists
    if [ -f "$ACTIVE_AREA_MEMORY" ]; then
        backup_name="/tmp/zed_area_memory_backup_$(date +%Y%m%d_%H%M%S).area"
        cp "$ACTIVE_AREA_MEMORY" "$backup_name"
        echo "💾 Current area memory backed up to: $backup_name"
    fi
    
    # Copy selected map to active location
    cp "$area_file" "$ACTIVE_AREA_MEMORY"
    echo "✅ Map '$map_name' loaded as active area memory"
    echo ""
    echo "🚀 Now you can start localization:"
    echo "   ros2 launch robot_bringup robot_localization.launch.py"
}

backup_area_memory() {
    if [ ! -f "$ACTIVE_AREA_MEMORY" ]; then
        echo "ℹ️  No active area memory to backup"
        return
    fi
    
    backup_name="/tmp/zed_area_memory_backup_$(date +%Y%m%d_%H%M%S).area"
    cp "$ACTIVE_AREA_MEMORY" "$backup_name"
    echo "💾 Area memory backed up to: $backup_name"
}

clear_area_memory() {
    if [ ! -f "$ACTIVE_AREA_MEMORY" ]; then
        echo "ℹ️  No active area memory to clear"
        return
    fi
    
    echo "⚠️  Clear active area memory?"
    echo "   This will remove: $ACTIVE_AREA_MEMORY"
    echo -n "   Continue? (y/N): "
    read -r response
    
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "❌ Clear cancelled"
        return
    fi
    
    # Backup before clearing
    backup_area_memory
    
    rm "$ACTIVE_AREA_MEMORY"
    echo "🧹 Active area memory cleared"
    echo ""
    echo "🚀 Ready for fresh mapping:"
    echo "   ros2 launch robot_bringup robot_mapping.launch.py"
}

# Main script logic
case "${1:-}" in
    "list"|"ls"|"")
        list_maps
        ;;
    "info"|"show")
        show_map_info "$2"
        ;;
    "delete"|"del"|"rm")
        delete_map "$2"
        ;;
    "load"|"activate")
        load_map "$2"
        ;;
    "backup")
        backup_area_memory
        ;;
    "clear"|"clean")
        clear_area_memory
        ;;
    "help"|"-h"|"--help")
        show_usage
        ;;
    *)
        echo "❌ Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac