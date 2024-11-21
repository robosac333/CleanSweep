#include "cleansweep/turtlebot.hpp"

TurtleBot::TurtleBot()
    : Node("turtlebot_node"),
      forwardSpeed(0.0),
      rotationSpeed(0.0),
      lastForwardSpeed(0.0),
      lastRotationSpeed(0.0),
      updateFrequency(10) {
    speedPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

TurtleBot::TurtleBot(float fSpeed, float rSpeed)
    : Node("turtlebot_node"),
      forwardSpeed(fSpeed),
      rotationSpeed(rSpeed),
      lastForwardSpeed(0.0),
      lastRotationSpeed(0.0),
      updateFrequency(10) {
    speedPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

TurtleBot::~TurtleBot() = default;

float TurtleBot::setForwardSpeed(float speed) {
    lastForwardSpeed = forwardSpeed;
    forwardSpeed = speed;
    return forwardSpeed;
}

float TurtleBot::setRotationSpeed(float speed) {
    lastRotationSpeed = rotationSpeed;
    rotationSpeed = speed;
    return rotationSpeed;
}

void TurtleBot::updatePosition(ObstacleAvoidance& obstacleAvoidance) {
    if (obstacleAvoidance.getObstacleStatus()) {
        // Stop if obstacle detected
        motionControl.linear.x = 0.0;
        motionControl.angular.z = 0.0;
    } else {
        // Move normally
        motionControl.linear.x = forwardSpeed;
        motionControl.angular.z = rotationSpeed;
    }
    
    speedPublisher->publish(motionControl);
}

bool TurtleBot::resetPosition() {
    forwardSpeed = 0.0;
    rotationSpeed = 0.0;
    lastForwardSpeed = 0.0;
    lastRotationSpeed = 0.0;
    
    motionControl.linear.x = 0.0;
    motionControl.angular.z = 0.0;
    speedPublisher->publish(motionControl);
    
    return true;
}

bool TurtleBot::verifySpeedChange() {
    return (forwardSpeed != lastForwardSpeed) || 
           (rotationSpeed != lastRotationSpeed);
}