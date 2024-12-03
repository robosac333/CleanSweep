// test_main.cpp
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    
    // Initialize ROS2
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }
    
    // Run all tests
    int result = RUN_ALL_TESTS();
    
    // Cleanup
    rclcpp::shutdown();
    
    return result;
}