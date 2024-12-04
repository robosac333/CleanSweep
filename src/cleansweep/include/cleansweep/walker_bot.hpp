#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cleansweep/object_detector.hpp"
#include <opencv2/core.hpp>
#include <algorithm>

// Forward declaration of WalkerState
class WalkerState;

class Walker : public rclcpp::Node {
public:
    Walker();

    void toggle_rotation_direction();
    double get_rotation_direction() const { return rotation_direction_; }

    // State management
    void change_state(WalkerState* new_state);

    // Velocity publishing
    void publish_velocity(double linear, double angular);

    // Sensor utilities
    bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

    // Object detection utilities
    bool is_red_object_detected() const { return red_object_detected_; }
    double get_object_distance() const { return object_distance_; }
    cv::Point2d get_object_center() const { return object_center_; }
    int get_image_width() const { return image_width_; }

    // Alignment utilities
    double get_alignment_threshold() const { return ALIGNMENT_THRESHOLD; }
    double calculate_angular_correction(double error) const;

    // Approach utilities
    double get_target_distance() const { return TARGET_DISTANCE; }

private:
    // Callback functions
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // State and publisher/subscriber
    WalkerState* current_state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

    // Constants
    const double SAFE_DISTANCE = 0.8;
    const double ALIGNMENT_THRESHOLD = 20.0;
    const double MAX_ANGULAR_SPEED = 0.3;
    const double TARGET_DISTANCE = 2.0;
    double rotation_direction_{1.0};



    // Object detection
    ObjectDetector object_detector_;
    bool red_object_detected_{false};
    double object_distance_{0.0};
    cv::Point2d object_center_;
    int image_width_{0};
};

// WalkerState base class
class WalkerState {
public:
    virtual ~WalkerState() = default;
    virtual void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

// ForwardState class
class ForwardState : public WalkerState {
public:
    void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

// RotationState class
class RotationState : public WalkerState {
public:
    void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) override;

private:
    bool initial_rotation_{true};
};

// AlignmentState class
class AlignmentState : public WalkerState {
public:
    void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

// ApproachState class
class ApproachState : public WalkerState {
public:
    void handle(Walker* walker, const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

#endif // WALKER_WALKER_BOT_HPP
