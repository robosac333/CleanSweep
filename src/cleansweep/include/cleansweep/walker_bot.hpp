#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP

#include <algorithm>  // for std::clamp
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cleansweep/object_detector.hpp"

// Add enum for obstacle location
enum class ObstacleLocation {
  NONE,
  LEFT,
  RIGHT,
  FRONT
};

class WalkerState;

class Walker : public rclcpp::Node {
  friend class WalkerBotTest; 
 public:
  Walker();
  void change_state(WalkerState* new_state);
  void publish_velocity(double linear, double angular);
  bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;
  void toggle_rotation_direction();
  void set_rotation_direction(double direction);  // New method
  ObstacleLocation detect_obstacle_location(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;  // New method
  double get_rotation_direction() const { return rotation_direction_; }
  rclcpp::TimerBase::SharedPtr create_timer(
      const std::chrono::duration<double>& period,
      std::function<void()> callback);
  bool is_red_object_detected() const { return red_object_detected_; }
  double get_object_distance() const { return object_distance_; }
  cv::Point2d get_object_center() const { return object_center_; }
  int get_image_width() const { return image_width_; }
  double get_alignment_threshold() const { return ALIGNMENT_THRESHOLD; }
  double calculate_angular_correction(double error) const;
  double get_target_distance() const { return TARGET_DISTANCE; }

 private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  WalkerState* current_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  double rotation_direction_;
  const double SAFE_DISTANCE = 1.0;
  const double TARGET_DISTANCE = 2.0;  // Distance to maintain from red object
  const double ALIGNMENT_THRESHOLD = 20.0;  // Pixels from center to consider aligned
  const double MAX_ANGULAR_SPEED = 0.3;
  ObjectDetector object_detector_;
  bool red_object_detected_;
  double object_distance_;
  cv::Point2d object_center_;
  int image_width_;
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
  RotationState() : initial_rotation_(true), rotation_timer_(nullptr) {}
  void handle(Walker* walker,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;

 private:
  bool initial_rotation_;
  rclcpp::TimerBase::SharedPtr rotation_timer_;
};

class AlignmentState : public WalkerState {
public:
    void handle(Walker* walker,
                [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

class ApproachState : public WalkerState {
public:
    ApproachState() : rotation_timer_(nullptr), is_rotating_(false) {}
    void handle(Walker* walker,
                [[maybe_unused]] const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
private:
    rclcpp::TimerBase::SharedPtr rotation_timer_;
    bool is_rotating_;
};

#endif  // WALKER_WALKER_BOT_HPP