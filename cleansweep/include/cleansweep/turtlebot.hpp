#ifndef INCLUDE_CLEANSWEEP_TURTLEBOT_HPP_
#define INCLUDE_CLEANSWEEP_TURTLEBOT_HPP_


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cleansweep/object_detection.hpp"
#include "cleansweep/obstacle_avoidance.hpp"

class TurtleBot : public rclcpp::Node {
 private:
  // Reorder members to match initialization order
  float forwardSpeed;
  float rotationSpeed;
  float lastForwardSpeed;
  float lastRotationSpeed;
  const int updateFrequency;
  
  /// Publisher for motion control commands
  geometry_msgs::msg::Twist motionControl;
  /// Publisher for robot speed
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speedPublisher;

 public:
  /**
   * @brief   Default constructor
   */
  TurtleBot();

  /**
   * @brief   Parameterized constructor
   * @param   fSpeed Initial forward speed
   * @param   rSpeed Initial rotation speed
   */
  TurtleBot(float fSpeed, float rSpeed);

  /**
   * @brief   Destructor
   */
  ~TurtleBot() override;

  float setForwardSpeed(float speed);
  float setRotationSpeed(float speed);
  void updatePosition(ObstacleAvoidance& obstacleAvoidance);
  bool resetPosition();
  bool verifySpeedChange();

  // Getters for testing
  float getForwardSpeed() const { return forwardSpeed; }
  float getRotationSpeed() const { return rotationSpeed; }
};

#endif // INCLUDE_CLEANSWEEP_TURTLEBOT_HPP_
