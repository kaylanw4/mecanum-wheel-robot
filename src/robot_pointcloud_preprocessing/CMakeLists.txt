cmake_minimum_required(VERSION 3.8)
project(robot_pointcloud_preprocessing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add compile definitions
add_definitions(${PCL_DEFINITIONS})

# Create executable directly with source files
add_executable(mapping_preprocessor_node
  src/mapping_preprocessor_node.cpp
  src/mapping_preprocessor.cpp
)

ament_target_dependencies(mapping_preprocessor_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
  tf2_geometry_msgs
)

target_link_libraries(mapping_preprocessor_node
  ${PCL_LIBRARIES}
)

target_include_directories(mapping_preprocessor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS
  mapping_preprocessor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()