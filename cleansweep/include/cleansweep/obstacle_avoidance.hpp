#ifndef INCLUDE_CLEANSWEEP_OBSTACLE_AVOIDANCE_HPP_
#define INCLUDE_CLEANSWEEP_OBSTACLE_AVOIDANCE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleAvoidance : public rclcpp::Node {
 private:
  /// Laser scan data subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sensorSubscriber;
  /// Flag to indicate if obstacle is present
  bool obstaclePresent;
  /// Movement speed
  float movementSpeed;

 public:
  /**
   * @brief  Default constructor
   * @param  none
   * @return none
   */
  ObstacleAvoidance();

  /**
   * @brief  Parameterized constructor
   * @param  speed Initial movement speed
   * @return none
   */
  explicit ObstacleAvoidance(float speed);

  /**
   * @brief  Destructor
   * @param  none
   * @return none
   */
  ~ObstacleAvoidance();

  /**
   * @brief  Detect if an obstacle is present
   * @param  none
   * @return bool True if obstacle detected
   */
  bool detectObstacle();

  /**
   * @brief  Set the obstacle detection status
   * @param  status Obstacle detection status
   * @return void
   */
  void setObstacleStatus(bool status);

  /**
   * @brief  Get the current obstacle status
   * @param  none
   * @return bool Current obstacle status
   */
  bool getObstacleStatus();

  /**
   * @brief  Process incoming laser scan data
   * @param  sensorData Pointer to laser scan data
   * @return void
   */
  void processSensorData(const sensor_msgs::msg::LaserScan::SharedPtr sensorData);
};

#endif  // INCLUDE_CLEANSWEEP_OBSTACLE_AVOIDANCE_HPP_