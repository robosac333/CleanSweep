#ifndef INCLUDE_TURTLEBOT_TURTLEBOT_H_
#define INCLUDE_TURTLEBOT_TURTLEBOT_H_

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "object_detection/object_detection.hpp"
#include "obstacle_avoidance/obstacle_avoidance.hpp"

class TurtleBot : public rclcpp::Node {
 private:
  /// Node handle for ROS2 communication
  rclcpp::Node::SharedPtr nodeId;
  /// Publisher for motion control commands
  geometry_msgs::msg::Twist motionControl;
  /// Publisher for robot speed
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speedPublisher;
  /// Current forward speed of the robot
  float forwardSpeed;
  /// Current rotation speed of the robot
  float rotationSpeed;
  /// Last recorded forward speed
  float lastForwardSpeed;
  /// Last recorded rotation speed
  float lastRotationSpeed;
  /// Update frequency for publishing commands
  const int updateFrequency;

 public:
  /**
   * @brief   Default constructor
   * @param   none
   * @return  none
   */
  TurtleBot();

  /**
   * @brief   Parameterized constructor
   * @param   forwardSpeed Initial forward speed
   * @param   rotationSpeed Initial rotation speed
   * @return  none
   */
  TurtleBot(float forwardSpeed, float rotationSpeed);

  /**
   * @brief   Destructor
   * @param   none
   * @return  none
   */
  ~TurtleBot();

  /**
   * @brief   Set the forward speed of the robot
   * @param   speed Forward speed value
   * @return  float Current forward speed
   */
  float setForwardSpeed(float speed);

  /**
   * @brief   Set the rotation speed of the robot
   * @param   speed Rotation speed value
   * @return  float Current rotation speed
   */
  float setRotationSpeed(float speed);

  /**
   * @brief   Update the robot's position considering obstacle avoidance
   * @param   obstacleAvoidance Reference to ObstacleAvoidance object
   * @return  void
   */
  void updatePosition(ObstacleAvoidance& obstacleAvoidance);

  /**
   * @brief   Reset the robot's position
   * @param   none
   * @return  bool Success status of reset operation
   */
  bool resetPosition();

  /**
   * @brief   Verify if there's been a change in speed
   * @param   none
   * @return  bool True if speed has changed, false otherwise
   */
  bool verifySpeedChange();
};

#endif  // INCLUDE_TURTLEBOT_TURTLEBOT_H_