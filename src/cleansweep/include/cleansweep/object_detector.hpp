#ifndef WALKER_OBJECT_DETECTOR_HPP
#define WALKER_OBJECT_DETECTOR_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

struct DetectionResult {
    bool detected;
    double distance; // Distance to object in meters
    cv::Point2d center; // Center of detected object
    cv::Mat debug_mask; // Store mask for visualization
    cv::Mat debug_hsv; // Store HSV image for visualization
};

class ObjectDetector {
public:
    ObjectDetector();
    DetectionResult detect_red_object(const sensor_msgs::msg::Image::SharedPtr msg);
private:
    // Color ranges for coke can detection in HSV
    const cv::Scalar COLOR_LOWER_LIMIT{170, 50, 50};
    const cv::Scalar COLOR_UPPER_LIMIT{255, 200, 90};
    const double DETECTION_AREA_THRESHOLD = 500.0;
    
    // Camera parameters (adjust these based on your camera)
    const double FOCAL_LENGTH = 525.0; // in pixels
    const double KNOWN_WIDTH = 0.3; // width of target object in meters
    const cv::Mat MORPH_KERNEL = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    
    void apply_morphological_operations(cv::Mat& mask) const;
    double estimate_distance(double pixel_width) const;
};
#endif // WALKER_OBJECT_DETECTOR_HPP
