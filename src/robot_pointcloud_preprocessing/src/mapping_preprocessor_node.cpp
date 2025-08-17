#include "robot_pointcloud_preprocessing/mapping_preprocessor.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<robot_pointcloud_preprocessing::MappingPreprocessor>();
    
    RCLCPP_INFO(node->get_logger(), "Starting MappingPreprocessor node...");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}