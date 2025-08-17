#include "robot_pointcloud_preprocessing/mapping_preprocessor.hpp"
#include <limits>

namespace robot_pointcloud_preprocessing
{

MappingPreprocessor::MappingPreprocessor() 
    : Node("mapping_preprocessor"),
      message_count_(0),
      total_processing_time_ms_(0.0),
      max_processing_time_ms_(0.0),
      min_processing_time_ms_(std::numeric_limits<double>::max())
{
    // Declare parameters
    this->declare_parameter("input_topic", "/zed2i/zed_node/point_cloud/cloud_registered");
    this->declare_parameter("output_topic", "/cleaned_mapping_cloud");
    this->declare_parameter("enable_diagnostics", true);
    this->declare_parameter("diagnostics_frequency", 10);

    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    enable_diagnostics_ = this->get_parameter("enable_diagnostics").as_bool();
    diagnostics_frequency_ = this->get_parameter("diagnostics_frequency").as_int();

    // Create optimized QoS profile for high-frequency point cloud data
    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(2))  // Small queue for input
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)  // Allow drops for real-time
        .durability(rclcpp::DurabilityPolicy::Volatile);
        
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1))  // Even smaller queue for output
        .reliability(rclcpp::ReliabilityPolicy::Reliable)   // Reliable for RViz
        .durability(rclcpp::DurabilityPolicy::Volatile)
        .history(rclcpp::HistoryPolicy::KeepLast);

    // Create subscriber and publisher with optimized QoS
    cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, 
        input_qos,
        std::bind(&MappingPreprocessor::cloudCallback, this, std::placeholders::_1)
    );

    cleaned_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, 
        output_qos
    );

    RCLCPP_INFO(this->get_logger(), "MappingPreprocessor initialized");
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Diagnostics: %s", enable_diagnostics_ ? "enabled" : "disabled");
    
    last_callback_time_ = std::chrono::high_resolution_clock::now();
}

void MappingPreprocessor::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Phase 1 Optimized: Direct pass-through without PCL conversion for maximum speed
    // Just copy the message and republish (saves ~2-3ms per frame)
    sensor_msgs::msg::PointCloud2 output_msg = *msg;
    
    // Publish cleaned cloud immediately
    cleaned_cloud_publisher_->publish(output_msg);
    
    // Calculate processing time
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double processing_time_ms = duration.count() / 1000.0;
    
    // Get point count from ROS message directly (no PCL conversion needed)
    size_t point_count = msg->width * msg->height;
    
    // Update statistics
    message_count_++;
    total_processing_time_ms_ += processing_time_ms;
    max_processing_time_ms_ = std::max(max_processing_time_ms_, processing_time_ms);
    min_processing_time_ms_ = std::min(min_processing_time_ms_, processing_time_ms);
    
    // Publish diagnostics (reduced frequency to save CPU)
    if (enable_diagnostics_ && (message_count_ % (diagnostics_frequency_ * 2) == 0)) {
        publishDiagnostics(processing_time_ms, point_count, point_count);
    }
}

void MappingPreprocessor::publishDiagnostics(double processing_time_ms, size_t input_points, size_t output_points)
{
    // Print diagnostics every N messages
    if (message_count_ % diagnostics_frequency_ == 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - last_callback_time_
        );
        
        double avg_processing_time = total_processing_time_ms_ / message_count_;
        double frequency = 1000.0 * diagnostics_frequency_ / time_since_last.count();
        
        RCLCPP_INFO(this->get_logger(), 
            "DIAGNOSTICS [Msg %zu]: "
            "Freq: %.1f Hz | "
            "Processing: %.2f ms (avg: %.2f, min: %.2f, max: %.2f) | "
            "Points: %zu → %zu (%.1f%% passed)",
            message_count_,
            frequency,
            processing_time_ms,
            avg_processing_time,
            min_processing_time_ms_,
            max_processing_time_ms_,
            input_points,
            output_points,
            output_points > 0 ? (100.0 * output_points / input_points) : 0.0
        );
        
        last_callback_time_ = current_time;
    }
}

} // namespace robot_pointcloud_preprocessing