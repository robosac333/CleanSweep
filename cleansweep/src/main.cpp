#include "rclcpp/rclcpp.hpp"
#include "cleansweep/object_detector.hpp"
#include "cleansweep/walker_bot.hpp"

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}
