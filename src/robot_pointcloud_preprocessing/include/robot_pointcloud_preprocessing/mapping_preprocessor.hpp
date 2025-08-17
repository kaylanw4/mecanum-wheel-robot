#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <chrono>

namespace robot_pointcloud_preprocessing
{

class MappingPreprocessor : public rclcpp::Node
{
public:
    MappingPreprocessor();
    ~MappingPreprocessor() = default;

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishDiagnostics(double processing_time_ms, size_t input_points, size_t output_points);

    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cleaned_cloud_publisher_;
    
    // Timing and diagnostics
    std::chrono::high_resolution_clock::time_point last_callback_time_;
    size_t message_count_;
    double total_processing_time_ms_;
    double max_processing_time_ms_;
    double min_processing_time_ms_;
    
    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    bool enable_diagnostics_;
    int diagnostics_frequency_;
};

} // namespace robot_pointcloud_preprocessing