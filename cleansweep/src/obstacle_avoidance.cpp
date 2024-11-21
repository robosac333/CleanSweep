#include "cleansweep/obstacle_avoidance.hpp"

ObstacleAvoidance::ObstacleAvoidance() 
    : Node("obstacle_avoidance_node"),
      obstaclePresent(false),
      movementSpeed(0.0) {
    
    sensorSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ObstacleAvoidance::processSensorData, this, std::placeholders::_1));
}

ObstacleAvoidance::ObstacleAvoidance(float speed) 
    : Node("obstacle_avoidance_node"),
      obstaclePresent(false),
      movementSpeed(speed) {
    
    sensorSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ObstacleAvoidance::processSensorData, this, std::placeholders::_1));
}

ObstacleAvoidance::~ObstacleAvoidance() {
}

bool ObstacleAvoidance::detectObstacle() {
    return obstaclePresent;  // Return actual status
}

void ObstacleAvoidance::setObstacleStatus(bool status) {
    obstaclePresent = status;
}

bool ObstacleAvoidance::getObstacleStatus() {
    return obstaclePresent;
}

void ObstacleAvoidance::processSensorData(
    const sensor_msgs::msg::LaserScan::SharedPtr sensorData) {
    // Basic implementation to avoid unused parameter warning
    if (sensorData->ranges.size() > 0) {
        // Check if any range reading is less than a threshold
        const float threshold = 0.5;  // 0.5 meters
        for (const float range : sensorData->ranges) {
            if (range < threshold) {
                setObstacleStatus(true);
                return;
            }
        }
        setObstacleStatus(false);
    }
}
// int main(int argc, char* argv[]) {
//     // Initialize ROS2
//     rclcpp::init(argc, argv);
    
//     // Create the obstacle avoidance node
//     auto node = std::make_shared<ObstacleAvoidance>();
    
//     // Spin the node
//     rclcpp::spin(node);
    
//     // Cleanup and shutdown
//     rclcpp::shutdown();
//     return 0;
// }