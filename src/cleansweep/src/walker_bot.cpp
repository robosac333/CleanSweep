/**
 * @file walker_bot.cpp
 * @brief Implementation of the Walker robot control system
 *
 * This file implements the Walker class and its associated state classes for
 * robot navigation, obstacle avoidance, and object tracking. The system uses
 * a state pattern to manage different behavioral modes and processes both
 * laser scan data for obstacle detection and camera images for object
 * detection.
 */

#include "cleansweep/walker_bot.hpp"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>

/**
 * @brief Construct a new Walker node
 *
 * Initializes the Walker node with publishers and subscribers for robot
 * control. Sets up velocity control, laser scan processing, and image
 * processing capabilities with appropriate QoS profiles and callback bindings.
 */
Walker::Walker() : Node("walker"), rotation_direction_(1.0), image_width_(0) {
  vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  auto qos_profile = rclcpp::QoS(10)
                         .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                         .history(rclcpp::HistoryPolicy::KeepLast);

  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile,
      std::bind(&Walker::scan_callback, this, std::placeholders::_1));

  image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10,
      std::bind(&Walker::image_callback, this, std::placeholders::_1));

  red_object_detected_ = false;
  current_state_ = new ForwardState();
}

/**
 * @brief Calculate angular correction for object alignment
 *
 * Computes the angular velocity needed to center an object in the camera view.
 * Normalizes the pixel error to the range [-1, 1] and scales it by
 * MAX_ANGULAR_SPEED.
 *
 * @param error Distance from image center in pixels
 * @return double Angular velocity correction, bounded by MAX_ANGULAR_SPEED
 */
double Walker::calculate_angular_correction(double error) const {
  double normalized_error = error / (image_width_ / 2.0);
  return std::clamp(normalized_error * MAX_ANGULAR_SPEED, -MAX_ANGULAR_SPEED,
                    MAX_ANGULAR_SPEED);
}

/**
 * @brief Process incoming camera images for object detection
 *
 * Handles image messages, performs red object detection, and updates robot
 * state based on detection results. Transitions to alignment state if an object
 * is detected while not already in alignment or approach states.
 *
 * @param msg Incoming image message
 */
void Walker::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  DetectionResult result = object_detector_.detect_red_object(msg);
  red_object_detected_ = result.detected;
  object_distance_ = result.distance;
  object_center_ = result.center;
  image_width_ = msg->width;

  if (red_object_detected_) {
    RCLCPP_INFO(this->get_logger(),
                "Red object detected at distance: %.2f meters, center_x: %.2f",
                object_distance_, object_center_.x);

    auto* current_alignment_state =
        dynamic_cast<AlignmentState*>(current_state_);
    auto* current_approach_state = dynamic_cast<ApproachState*>(current_state_);

    if (current_alignment_state == nullptr &&
        current_approach_state == nullptr) {
      change_state(new AlignmentState());
    }
  }
}

/**
 * @brief Process incoming laser scan data
 *
 * Delegates scan processing to the current state handler for appropriate
 * behavior based on the robot's current state.
 *
 * @param scan Incoming laser scan message
 */
void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  current_state_->handle(this, scan);
}

/**
 * @brief Change the current state of the robot
 *
 * Manages state transitions by safely deleting the old state and setting the
 * new one. Handles memory management for state objects.
 *
 * @param new_state Pointer to the new state object
 */
void Walker::change_state(WalkerState* new_state) {
  delete current_state_;
  current_state_ = new_state;
}

/**
 * @brief Publish velocity commands to the robot
 *
 * Creates and publishes a Twist message with the specified linear and angular
 * velocities.
 *
 * @param linear Linear velocity in m/s
 * @param angular Angular velocity in rad/s
 */
void Walker::publish_velocity(double linear, double angular) {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_publisher_->publish(msg);
}

/**
 * @brief Detect the location of obstacles relative to the robot
 *
 * Analyzes laser scan data to determine if obstacles are present in the
 * left, right, or front sectors of the robot. Uses defined angle ranges
 * and the SAFE_DISTANCE threshold for detection.
 *
 * @param scan Laser scan data
 * @return ObstacleLocation Enumerated location of detected obstacle
 */
ObstacleLocation Walker::detect_obstacle_location(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  const int left_start = 0;
  const int left_end = 17;
  const int right_start = 343;
  const int right_end = 359;
  const int front_start = 18;
  const int front_end = 342;

  bool left_blocked = false;
  bool right_blocked = false;
  bool front_blocked = false;

  // Check left arc
  for (int i = left_start; i <= left_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      left_blocked = true;
      RCLCPP_INFO(rclcpp::get_logger("Walker"),
                  "Obstacle detected in left arc at angle %d", i);
      break;
    }
  }

  // Check right arc
  for (int i = right_start; i <= right_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      right_blocked = true;
      RCLCPP_INFO(rclcpp::get_logger("Walker"),
                  "Obstacle detected in right arc at angle %d", i);
      break;
    }
  }

  // Check front arc
  for (int i = front_start; i <= front_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      front_blocked = true;
      RCLCPP_INFO(rclcpp::get_logger("Walker"),
                  "Obstacle detected in front at angle %d", i);
      break;
    }
  }

  if (left_blocked && !right_blocked && !front_blocked) {
    return ObstacleLocation::LEFT;
  } else if (!left_blocked && right_blocked && !front_blocked) {
    return ObstacleLocation::RIGHT;
  } else if (front_blocked || (left_blocked && right_blocked)) {
    return ObstacleLocation::FRONT;
  }

  return ObstacleLocation::NONE;
}

/**
 * @brief Check if the robot's path is clear of obstacles
 *
 * Wrapper around detect_obstacle_location that returns true only if
 * no obstacles are detected in any sector.
 *
 * @param scan Laser scan data
 * @return bool True if path is clear, false if any obstacles are detected
 */
bool Walker::is_path_clear(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  return detect_obstacle_location(scan) == ObstacleLocation::NONE;
}

/**
 * @brief Toggle the robot's rotation direction
 *
 * Inverts the current rotation direction by multiplying by -1.
 */
void Walker::toggle_rotation_direction() { rotation_direction_ *= -1.0; }

/**
 * @brief Set the robot's rotation direction
 *
 * @param direction Desired rotation direction (-1.0 for CCW, 1.0 for CW)
 */
void Walker::set_rotation_direction(double direction) {
  rotation_direction_ = direction;
}

/**
 * @brief Create a timer for state-specific timing needs
 *
 * Wrapper around the ROS timer creation to maintain consistent timing across
 * states.
 *
 * @param period Timer period
 * @param callback Timer callback function
 * @return rclcpp::TimerBase::SharedPtr Created timer
 */
rclcpp::TimerBase::SharedPtr Walker::create_timer(
    const std::chrono::duration<double>& period,
    std::function<void()> callback) {
  return this->create_wall_timer(period, callback);
}
/**
 * @brief Handle robot behavior in the alignment state
 *
 * Controls the robot to align its center with a detected object by adjusting
 * angular velocity. Transitions to approach state when aligned or forward state
 * if object is lost.
 *
 * @param walker Pointer to the Walker instance
 * @param scan Current laser scan data (unused in this state)
 */
void AlignmentState::handle(
    Walker* walker,
    [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (!walker->is_red_object_detected()) {
    RCLCPP_INFO(walker->get_logger(),
                "Lost object detection, returning to forward state");
    walker->change_state(new ForwardState());
    return;
  }

  double image_center_x = walker->get_image_width() / 2.0;
  double object_center_x = walker->get_object_center().x;
  double center_error = object_center_x - image_center_x;

  if (std::abs(center_error) < walker->get_alignment_threshold()) {
    RCLCPP_INFO(walker->get_logger(),
                "Object centered, switching to approach state");
    walker->change_state(new ApproachState());
    return;
  }

  double angular_velocity = walker->calculate_angular_correction(center_error);
  walker->publish_velocity(0.0, -angular_velocity);

  RCLCPP_INFO(
      walker->get_logger(),
      "Aligning with object. Center error: %.2f, Angular velocity: %.2f",
      center_error, angular_velocity);
}

/**
 * @brief Handle robot behavior in the approach state
 *
 * Controls the robot to approach a detected object while maintaining alignment.
 * Performs a celebratory rotation when reaching the target distance and
 * transitions back to forward state. Manages approach timing and rotation
 * completion through a timer system.
 *
 * @param walker Pointer to the Walker instance
 * @param scan Current laser scan data (unused in this state)
 */
void ApproachState::handle(
    Walker* walker,
    [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (is_rotating_) {
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
    return;
  }

  if (!walker->is_red_object_detected()) {
    RCLCPP_INFO(
        walker->get_logger(),
        "Lost object detection during approach, returning to forward state");
    walker->change_state(new ForwardState());
    return;
  }

  if (walker->get_object_distance() <= walker->get_target_distance()) {
    if (!rotation_timer_) {
      RCLCPP_INFO(walker->get_logger(),
                  "Reached target distance, starting 4-second rotation");

      is_rotating_ = true;

      rotation_timer_ =
          walker->create_timer(std::chrono::seconds(4), [walker]() {
            RCLCPP_INFO(
                walker->get_logger(),
                "4-second rotation complete, changing to forward state");
            walker->change_state(new ForwardState());
          });
    }

    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
    return;
  }

  walker->publish_velocity(0.5, 0.0);
  RCLCPP_INFO(
      walker->get_logger(),
      "Approaching object. Current distance: %.2f meters, Target: %.2f meters",
      walker->get_object_distance(), walker->get_target_distance());
}

/**
 * @brief Handle robot behavior in the forward state
 *
 * Controls forward movement and obstacle avoidance. Transitions to alignment
 * state if an object is detected, or rotation state if obstacles are
 * encountered. Implements the primary navigation behavior of the robot.
 *
 * @param walker Pointer to the Walker instance
 * @param scan Current laser scan data
 */
void ForwardState::handle(Walker* walker,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_red_object_detected()) {
    walker->change_state(new AlignmentState());
    return;
  }

  ObstacleLocation obstacle = walker->detect_obstacle_location(scan);

  switch (obstacle) {
    case ObstacleLocation::NONE:
      walker->publish_velocity(0.5, 0.0);
      break;

    case ObstacleLocation::LEFT:
      walker->set_rotation_direction(-1.0);
      RCLCPP_INFO(walker->get_logger(),
                  "Left obstacle detected, rotating right");
      walker->change_state(new RotationState());
      break;

    case ObstacleLocation::RIGHT:
      walker->set_rotation_direction(1.0);
      RCLCPP_INFO(walker->get_logger(),
                  "Right obstacle detected, rotating left");
      walker->change_state(new RotationState());
      break;

    case ObstacleLocation::FRONT:
      walker->set_rotation_direction(1.0);
      RCLCPP_INFO(walker->get_logger(),
                  "Front obstacle detected, rotating left");
      walker->change_state(new RotationState());
      break;
  }
}

/**
 * @brief Handle robot behavior in the rotation state
 *
 * Controls rotational movement for obstacle avoidance. Implements both initial
 * timed rotation and continuous rotation until a clear path is found or an
 * object is detected. Uses a timer system to manage the initial rotation
 * period.
 *
 * @param walker Pointer to the Walker instance
 * @param scan Current laser scan data
 */
void RotationState::handle(Walker* walker,
                           const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_red_object_detected()) {
    walker->change_state(new AlignmentState());
    return;
  }

  if (initial_rotation_) {
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());

    if (!rotation_timer_) {
      rotation_timer_ = walker->create_timer(std::chrono::seconds(5), [this]() {
        initial_rotation_ = false;
        rotation_timer_ = nullptr;
      });

      RCLCPP_INFO(walker->get_logger(),
                  "Starting initial 5-second rotation period");
    }
  } else {
    ObstacleLocation obstacle = walker->detect_obstacle_location(scan);

    if (obstacle == ObstacleLocation::NONE) {
      RCLCPP_INFO(walker->get_logger(), "Path is clear, moving forward");
      walker->change_state(new ForwardState());
    } else {
      walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
      RCLCPP_INFO(walker->get_logger(), "Path blocked, continuing rotation");
    }
  }
}
