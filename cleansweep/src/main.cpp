#include "rclcpp/rclcpp.hpp"
#include "cleansweep/object_detection.hpp"
#include "cleansweep/obstacle_avoidance.hpp"
#include "cleansweep/turtlebot.hpp"

int main(int argc, char* argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Clean shutdown
    rclcpp::shutdown();
    return 0;
}
