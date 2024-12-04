#include "cleansweep/walker_bot.hpp"

Walker::Walker() : Node("walker") {
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Walker::scan_callback, this, std::placeholders::_1));
    
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&Walker::image_callback, this, std::placeholders::_1));

    current_state_ = new ForwardState();
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
    for (int i = 0; i < 17; ++i) {
        if (scan->ranges[i] < SAFE_DISTANCE) return false;
    }
    for (int i = 343; i < 360; ++i) {
        if (scan->ranges[i] < SAFE_DISTANCE) return false;
    }
    return true;
}

double Walker::calculate_angular_correction(double error) const {
    double normalized_error = error / (image_width_ / 2.0);
    return std::clamp(normalized_error * MAX_ANGULAR_SPEED, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
}

void ForwardState::handle(Walker* walker,
                         const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (walker->is_path_clear(scan)) {
        walker->publish_velocity(0.5, 0.0);
    } else {
        walker->change_state(new RotationState());
    }
}

void RotationState::handle(Walker* walker,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (walker->is_path_clear(scan)) {
        walker->change_state(new ForwardState());
    } else {
        walker->publish_velocity(0.0, 0.3);
    }
}


void AlignmentState::handle(Walker* walker,
                           const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!walker->is_red_object_detected()) {
        walker->change_state(new ForwardState());
        return;
    }

    double image_center_x = walker->get_image_width() / 2.0;
    double object_center_x = walker->get_object_center().x;
    double center_error = object_center_x - image_center_x;

    if (std::abs(center_error) < walker->get_alignment_threshold()) {
        walker->change_state(new ForwardState());
        return;
    }

    double angular_velocity = walker->calculate_angular_correction(center_error);
    walker->publish_velocity(0.0, -angular_velocity);
}

void ApproachState::handle(Walker* walker,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!walker->is_red_object_detected()) {
        walker->change_state(new ForwardState());
        return;
    }

    if (walker->get_object_distance() <= walker->get_target_distance()) {
        walker->publish_velocity(0.0, 0.0);
        return;
    }

    walker->publish_velocity(0.5, 0.0);
}

void Walker::toggle_rotation_direction() {
    rotation_direction_ *= -1.0;
}