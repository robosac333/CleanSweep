#include "cleansweep/walker_bot.hpp"

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

bool Walker::is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  const int left_start = 0;
  const int left_end = 17;
  const int right_start = 343;
  const int right_end = 359;

  for (int i = left_start; i <= left_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(rclcpp::get_logger("Walker"),
                  "Obstacle detected in left arc at angle %d", i);
      return false;
    }
  }

  for (int i = right_start; i <= right_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(rclcpp::get_logger("Walker"),
                  "Obstacle detected in right arc at angle %d", i);
      return false;
    }
  }
  return true;
}

void Walker::toggle_rotation_direction() {
  rotation_direction_ *= -1.0;
}

rclcpp::TimerBase::SharedPtr Walker::create_timer(
    const std::chrono::duration<double>& period,
    std::function<void()> callback) {
  return this->create_wall_timer(period, callback);
}

void AlignmentState::handle(Walker* walker,
                           const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // First check if path is clear
  if (!walker->is_path_clear(scan)) {
    RCLCPP_INFO(walker->get_logger(), "Obstacle detected during alignment, switching to rotation state");
    walker->toggle_rotation_direction();
    walker->change_state(new RotationState());
    return;
  }

  // Then check if we still see the object
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
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // First check if path is clear
  if (!walker->is_path_clear(scan)) {
    RCLCPP_INFO(walker->get_logger(), "Obstacle detected during approach, switching to rotation state");
    walker->toggle_rotation_direction();
    walker->change_state(new RotationState());
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
    RCLCPP_INFO(walker->get_logger(), 
                "Reached target distance (%.2f meters), stopping",
                walker->get_object_distance());
    walker->publish_velocity(0.0, 0.0);
    return;
  }

  // Move forward at constant speed
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

  if (walker->is_path_clear(scan)) {
    walker->publish_velocity(0.5, 0.0);
  } else {
    walker->toggle_rotation_direction();
    RCLCPP_INFO(
        walker->get_logger(),
        "Obstacle detected, changing rotation direction and starting rotation");
    walker->change_state(new RotationState());
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
    if (walker->is_path_clear(scan)) {
      RCLCPP_INFO(walker->get_logger(), "Path is clear, moving forward");
      walker->change_state(new ForwardState());
    } else {
      walker->publish_velocity(0.0, 0.3 * walker->get_rotation_direction());
      RCLCPP_INFO(walker->get_logger(), "Path blocked, continuing rotation");
    }
  }
}
