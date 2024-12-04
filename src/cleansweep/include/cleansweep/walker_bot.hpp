#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cleansweep/object_detector.hpp"

class WalkerState;

class Walker : public rclcpp::Node {
public:
    Walker();
    void change_state(WalkerState* new_state);
    void publish_velocity(double linear, double angular);
    bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

    bool is_red_object_detected() const { return red_object_detected_; }
    double get_object_distance() const { return object_distance_; }
    cv::Point2d get_object_center() const { return object_center_; }
    int get_image_width() const { return image_width_; }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    WalkerState* current_state_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    const double SAFE_DISTANCE = 0.8;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    ObjectDetector object_detector_;
    bool red_object_detected_{false};
    double object_distance_{0.0};
    cv::Point2d object_center_;
    int image_width_{0};
  
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

class WalkerState {
public:
    virtual ~WalkerState() = default;
    virtual void handle(Walker* walker,
                       const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

class ForwardState : public WalkerState {
public:
    void handle(Walker* walker,
                const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

class RotationState : public WalkerState {
public:
    void handle(Walker* walker,
                const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
private:
    bool initial_rotation_{true};
};

#endif // WALKER_WALKER_BOT_HPP
