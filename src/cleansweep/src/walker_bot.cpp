#include "cleansweep/walker_bot.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>

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

double Walker::calculate_angular_correction(double error) const {
  // Normalize error by image width to get a proportion (-1 to 1)
  double normalized_error = error / (image_width_ / 2.0);
  // Scale the correction by MAX_ANGULAR_SPEED and ensure it's within bounds
  return std::clamp(normalized_error * MAX_ANGULAR_SPEED, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
}

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
    
    // Only change to alignment state if we're in Forward or Rotation state
    auto* current_alignment_state = dynamic_cast<AlignmentState*>(current_state_);
    auto* current_approach_state = dynamic_cast<ApproachState*>(current_state_);
    
    if (current_alignment_state == nullptr && current_approach_state == nullptr) {
      change_state(new AlignmentState());
    }
  }
}

void Walker::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  current_state_->handle(this, scan);
}

void Walker::change_state(WalkerState* new_state) {
  delete current_state_;
  current_state_ = new_state;
}

void Walker::publish_velocity(double linear, double angular) {
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;
  msg.angular.z = angular;
  vel_publisher_->publish(msg);
}

ObstacleLocation Walker::detect_obstacle_location(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
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

  // Determine overall obstacle location
  if (left_blocked && !right_blocked && !front_blocked) {
    return ObstacleLocation::LEFT;
  } else if (!left_blocked && right_blocked && !front_blocked) {
    return ObstacleLocation::RIGHT;
  } else if (front_blocked || (left_blocked && right_blocked)) {
    return ObstacleLocation::FRONT;
  }
  
  return ObstacleLocation::NONE;
}

bool Walker::is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  return detect_obstacle_location(scan) == ObstacleLocation::NONE;
}

void Walker::toggle_rotation_direction() {
  rotation_direction_ *= -1.0;
}

void Walker::set_rotation_direction(double direction) {
  rotation_direction_ = direction;
}

rclcpp::TimerBase::SharedPtr Walker::create_timer(
    const std::chrono::duration<double>& period,
    std::function<void()> callback) {
  return this->create_wall_timer(period, callback);
}

void AlignmentState::handle(Walker* walker,
                           [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Check if we still see the object
  if (!walker->is_red_object_detected()) {
    RCLCPP_INFO(walker->get_logger(), "Lost object detection, returning to forward state");
    walker->change_state(new ForwardState());
    return;
  }

  double image_center_x = walker->get_image_width() / 2.0;
  double object_center_x = walker->get_object_center().x;
  
  // Calculate the error (how far the object is from the center)
  double center_error = object_center_x - image_center_x;

  // Check if we're aligned within threshold
  if (std::abs(center_error) < walker->get_alignment_threshold()) {
    RCLCPP_INFO(walker->get_logger(), "Object centered, switching to approach state");
    walker->change_state(new ApproachState());
    return;
  }

  // Calculate and apply rotation correction
  double angular_velocity = walker->calculate_angular_correction(center_error);
  walker->publish_velocity(0.0, -angular_velocity);
  
  RCLCPP_INFO(walker->get_logger(), 
              "Aligning with object. Center error: %.2f, Angular velocity: %.2f",
              center_error, angular_velocity);
}

void ApproachState::handle(Walker* walker,
                          [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // If we're already rotating, just continue rotating until timer expires
  if (is_rotating_) {
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
    return;
  }

  // Check if we still see the object
  if (!walker->is_red_object_detected()) {
    RCLCPP_INFO(walker->get_logger(), "Lost object detection during approach, returning to forward state");
    walker->change_state(new ForwardState());
    return;
  }

  // Check if we've reached the target distance
  if (walker->get_object_distance() <= walker->get_target_distance()) {
    if (!rotation_timer_) {
      // Start the rotation and create the timer
      RCLCPP_INFO(walker->get_logger(), 
                  "Reached target distance, starting 4-second rotation");
      
      is_rotating_ = true;  // Set the rotating flag
      
      rotation_timer_ = walker->create_timer(
          std::chrono::seconds(4),
          [this, walker]() {
            RCLCPP_INFO(walker->get_logger(), "4-second rotation complete, changing to forward state");
            walker->change_state(new ForwardState());
          });
    }
    
    // Keep rotating
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
    return;
  }

  // If not at target distance yet, keep moving forward
  walker->publish_velocity(0.5, 0.0);
  RCLCPP_INFO(walker->get_logger(), 
              "Approaching object. Current distance: %.2f meters, Target: %.2f meters",
              walker->get_object_distance(),
              walker->get_target_distance());
}

void ForwardState::handle(Walker* walker,
                         const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_red_object_detected()) {
    walker->change_state(new AlignmentState());
    return;
  }

  ObstacleLocation obstacle = walker->detect_obstacle_location(scan);
  
  switch(obstacle) {
    case ObstacleLocation::NONE:
      walker->publish_velocity(0.5, 0.0);
      break;
      
    case ObstacleLocation::LEFT:
      // Set rotation direction to right (negative)
      walker->set_rotation_direction(-1.0);
      RCLCPP_INFO(walker->get_logger(),
                  "Left obstacle detected, rotating right");
      walker->change_state(new RotationState());
      break;
      
    case ObstacleLocation::RIGHT:
      // Set rotation direction to left (positive)
      walker->set_rotation_direction(1.0);
      RCLCPP_INFO(walker->get_logger(),
                  "Right obstacle detected, rotating left");
      walker->change_state(new RotationState());
      break;
      
    case ObstacleLocation::FRONT:
      // Keep current rotation direction or set default
      walker->set_rotation_direction(1.0);
      RCLCPP_INFO(walker->get_logger(),
                  "Front obstacle detected, rotating left");
      walker->change_state(new RotationState());
      break;
  }
}

void RotationState::handle(Walker* walker,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (walker->is_red_object_detected()) {
    walker->change_state(new AlignmentState());
    return;
  }

  if (initial_rotation_) {
    walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());

    if (!rotation_timer_) {
      rotation_timer_ =
          walker->create_timer(std::chrono::seconds(5), [this]() {
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
      // Continue rotating in the current direction
      walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
      RCLCPP_INFO(walker->get_logger(), "Path blocked, continuing rotation");
    }
  }
}