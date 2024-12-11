#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP

#include <algorithm>

#include "cleansweep/object_detector.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @brief Enumeration of possible obstacle locations relative to the robot
 */
enum class ObstacleLocation {
  NONE,   ///< No obstacle detected
  LEFT,   ///< Obstacle detected on the left
  RIGHT,  ///< Obstacle detected on the right
  FRONT   ///< Obstacle detected in front
};

// Forward declaration
class WalkerState;

/**
 * @brief Main robot control node implementing wandering and object detection
 * behavior
 *
 * This class manages the robot's movement, obstacle avoidance, and object
 * detection. It uses a state pattern to handle different behavioral states and
 * processes both laser scan and image data for navigation and object detection.
 */
class Walker : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Walker node
   */
  Walker();

  friend class WalkerBotTest;  ///< Friend declaration for testing

  /**
   * @brief Get the current state of the walker
   * @return WalkerState* Pointer to the current state object
   */
  WalkerState* get_current_state() const { return current_state_; }

  /**
   * @brief Process a laser scan message (used for testing)
   * @param scan The laser scan message to process
   */
  void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    scan_callback(scan);
  }

  /**
   * @brief Process an image message (used for testing)
   * @param msg The image message to process
   */
  void process_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    image_callback(msg);
  }

  /**
   * @brief Change the current state of the walker
   * @param new_state Pointer to the new state to transition to
   */
  void change_state(WalkerState* new_state);

  /**
   * @brief Publish velocity commands to the robot
   * @param linear Linear velocity in m/s
   * @param angular Angular velocity in rad/s
   */
  void publish_velocity(double linear, double angular);

  /**
   * @brief Check if the robot's path is clear of obstacles
   * @param scan Current laser scan data
   * @return bool True if path is clear, false otherwise
   */
  bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

  /**
   * @brief Toggle the direction of rotation
   */
  void toggle_rotation_direction();

  /**
   * @brief Set the rotation direction
   * @param direction Direction of rotation (-1 for CCW, 1 for CW)
   */
  void set_rotation_direction(double direction);

  /**
   * @brief Detect the location of obstacles relative to the robot
   * @param scan Current laser scan data
   * @return ObstacleLocation Location of detected obstacle
   */
  ObstacleLocation detect_obstacle_location(
      const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

  /**
   * @brief Get the current rotation direction
   * @return double Current rotation direction (-1 or 1)
   */
  double get_rotation_direction() const { return rotation_direction_; }

  /**
   * @brief Create a timer for state-specific timing needs
   * @param period Timer period
   * @param callback Timer callback function
   * @return rclcpp::TimerBase::SharedPtr Created timer
   */
  rclcpp::TimerBase::SharedPtr create_timer(
      const std::chrono::duration<double>& period,
      std::function<void()> callback);

  // Getter methods with inline documentation
  bool is_red_object_detected() const {
    return red_object_detected_;
  }  ///< Check if a red object is currently detected
  double get_object_distance() const {
    return object_distance_;
  }  ///< Get distance to detected object
  cv::Point2d get_object_center() const {
    return object_center_;
  }  ///< Get center position of detected object
  int get_image_width() const {
    return image_width_;
  }  ///< Get width of processed image
  double get_alignment_threshold() const {
    return ALIGNMENT_THRESHOLD;
  }  ///< Get threshold for object alignment
  double get_target_distance() const {
    return TARGET_DISTANCE;
  }  ///< Get target distance for object approach
  double get_max_angular_speed() const {
    return MAX_ANGULAR_SPEED;
  }  ///< Get maximum angular speed

  /**
   * @brief Calculate angular correction based on alignment error
   * @param error Current alignment error
   * @return double Angular correction value
   */
  double calculate_angular_correction(double error) const;

 protected:
  /**
   * @brief Callback for processing laser scan messages
   * @param scan Incoming laser scan message
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief Callback for processing image messages
   * @param msg Incoming image message
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  WalkerState* current_state_;  ///< Current state of the walker

 private:
  // ROS publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  // Member variables
  double rotation_direction_;          ///< Current rotation direction (-1 or 1)
  const double SAFE_DISTANCE = 1.0;    ///< Minimum safe distance to obstacles
  const double TARGET_DISTANCE = 2.0;  ///< Target distance for object approach
  const double ALIGNMENT_THRESHOLD = 20.0;  ///< Threshold for object alignment
  const double MAX_ANGULAR_SPEED = 0.3;     ///< Maximum angular speed

  ObjectDetector object_detector_;  ///< Object detector instance
  bool red_object_detected_;        ///< Flag for red object detection
  double object_distance_;          ///< Distance to detected object
  cv::Point2d object_center_;       ///< Center of detected object
  int image_width_;                 ///< Width of processed image
};

/**
 * @brief Abstract base class for walker states
 *
 * Defines the interface for different behavioral states of the walker.
 */
class WalkerState {
 public:
  virtual ~WalkerState() = default;

  /**
   * @brief Handle the current state
   * @param walker Pointer to the walker instance
   * @param scan Current laser scan data
   */
  virtual void handle(Walker* walker,
                      const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

/**
 * @brief State for moving forward
 *
 * Handles forward movement and obstacle detection.
 */
class ForwardState : public WalkerState {
 public:
  void handle(Walker* walker,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

/**
 * @brief State for rotating in place
 *
 * Handles rotation when obstacles are detected or during exploration.
 */
class RotationState : public WalkerState {
 public:
  RotationState() : initial_rotation_(true), rotation_timer_(nullptr) {}
  void handle(Walker* walker,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;

 private:
  bool initial_rotation_;                        ///< Flag for initial rotation
  rclcpp::TimerBase::SharedPtr rotation_timer_;  ///< Timer for rotation control
};

/**
 * @brief State for aligning with detected objects
 *
 * Handles alignment of the robot with detected objects.
 */
class AlignmentState : public WalkerState {
 public:
  void handle(Walker* walker,
              [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr
                  scan) override;
};

/**
 * @brief State for approaching detected objects
 *
 * Handles movement towards detected objects while maintaining alignment.
 */
class ApproachState : public WalkerState {
 public:
  ApproachState() : rotation_timer_(nullptr), is_rotating_(false) {}
  void handle(Walker* walker,
              [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr
                  scan) override;

 private:
  rclcpp::TimerBase::SharedPtr rotation_timer_;  ///< Timer for rotation control
  bool is_rotating_;  ///< Flag indicating rotation status
};

#endif  // WALKER_WALKER_BOT_HPP
