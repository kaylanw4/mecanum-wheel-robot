# Point Cloud Preprocessing Pipeline for ZED2i Camera

## Overview
This package implements a multi-phase point cloud preprocessing pipeline to clean ZED2i stereo camera data before SLAM fusion. The pipeline addresses issues with corrupted point cloud data from reflective surfaces like mirrors and windows.

## Phase 1: Basic Infrastructure ✅ COMPLETED

### Implementation Status
- ✅ ROS2 Python package structure
- ✅ Pass-through preprocessing node
- ✅ Configurable parameters
- ✅ Performance monitoring and statistics
- ✅ Launch file integration
- ✅ RViz visualization configuration
- ✅ Package builds successfully

### Current Features
- **Pass-through processing**: Data flows unchanged to validate infrastructure
- **Performance monitoring**: Processing time statistics (avg/min/max)
- **Configurable topics**: Input/output topic remapping via parameters
- **Real-time statistics**: 10-second interval reporting
- **RViz visualization**: Side-by-side comparison of raw vs processed clouds

### Usage

#### Standalone Testing
```bash
# Terminal 1: Start preprocessing node only
source install/setup.bash
ros2 run robot_pointcloud_preprocessing pointcloud_preprocessor

# Terminal 2: Check node status
ros2 topic list | grep cloud
ros2 node info /pointcloud_preprocessor
```

#### Full Robot Integration Testing
```bash
# Launch robot with ZED2i + preprocessing + visualization
source install/setup.bash
ros2 launch robot_pointcloud_preprocessing robot_with_preprocessing.launch.py

# Or launch just preprocessing (if ZED2i already running)
ros2 launch robot_pointcloud_preprocessing pointcloud_preprocessing.launch.py
```

#### Manual Testing with Real Data
```bash
# Terminal 1: Start ZED2i camera
ros2 launch robot_bringup robot_with_zed2i.launch.py use_rviz:=false

# Terminal 2: Start preprocessing with visualization
ros2 launch robot_pointcloud_preprocessing pointcloud_preprocessing.launch.py
```

### Testing Results Phase 1
- [x] Package builds without errors
- [x] Node initializes with correct parameters
- [x] Topics are configured correctly (/zed2i/zed_node/point_cloud/cloud_registered → /cleaned_point_cloud)
- [x] Performance monitoring enabled
- [x] Launch files work correctly
- [ ] **PENDING**: End-to-end testing with actual ZED2i data
- [ ] **PENDING**: RViz visualization validation
- [ ] **PENDING**: Performance benchmarking (<10ms latency requirement)

### Configuration Files
- `config/preprocessing_params.yaml` - Main configuration parameters
- `config/preprocessing_view.rviz` - RViz visualization setup
- `launch/pointcloud_preprocessing.launch.py` - Standalone preprocessing
- `launch/robot_with_preprocessing.launch.py` - Full robot integration

### Key Topics
- **Input**: `/zed2i/zed_node/point_cloud/cloud_registered` (sensor_msgs/PointCloud2)
- **Output**: `/cleaned_point_cloud` (sensor_msgs/PointCloud2)

## Phase 2: Statistical Outlier Removal (NOT YET IMPLEMENTED)
- k-nearest neighbor outlier detection
- Configurable k and stddev_threshold parameters
- Remove statistical outliers while preserving good geometry

## Phase 3: Confidence-Based Filtering (NOT YET IMPLEMENTED)
- ZED2i confidence map integration
- Low-confidence point filtering
- Topic synchronization between point cloud and confidence data

## Phase 4: Temporal Consistency (NOT YET IMPLEMENTED)
- Multi-frame point stability tracking
- 3-5 frame temporal buffer
- Remove phantom points that don't persist across frames

## Phase 5: Mirror Detection (NOT YET IMPLEMENTED)
- RANSAC plane detection for large flat surfaces
- Normal vector analysis for surface consistency
- Intensity-based reflection detection
- Phantom geometry validation

## Phase 6: SLAM Integration (NOT YET IMPLEMENTED)
- Integration with existing SLAM pipeline
- Depth-to-scan conversion using cleaned data
- Performance optimization for real-time operation

## Performance Targets
- **Latency**: <10ms total preprocessing time per frame
- **Throughput**: Handle 60Hz ZED2i input
- **Memory**: Reasonable memory usage for Jetson Orin Nano

## Development Notes
- Each phase is implemented and tested independently
- Parameters are configurable via YAML files
- ROS2 best practices followed throughout
- Clean error handling and logging
- Proper QoS matching with ZED2i publisher (BEST_EFFORT)