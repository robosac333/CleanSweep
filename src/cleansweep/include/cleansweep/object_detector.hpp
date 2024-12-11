#ifndef WALKER_OBJECT_DETECTOR_HPP
#define WALKER_OBJECT_DETECTOR_HPP

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @brief Struct containing the results of object detection
 *
 * This struct holds all relevant information about a detected object,
 * including its presence, distance, position, and debug visualization data.
 */
struct DetectionResult {
  bool detected;       ///< Flag indicating whether an object was detected
  double distance;     ///< Distance to object in meters
  cv::Point2d center;  ///< Center coordinates of detected object
  cv::Mat debug_mask;  ///< Binary mask used for object detection (for
                       ///< visualization)
  cv::Mat debug_hsv;   ///< HSV converted image (for visualization)
};

/**
 * @brief Class for detecting and analyzing objects in images
 *
 * This class provides functionality to detect red objects (e.g., coke cans)
 * in images using HSV color space filtering and morphological operations.
 * It also estimates the distance to detected objects using known object
 * dimensions.
 */
class ObjectDetector {
 public:
  /**
   * @brief Construct a new Object Detector object
   */
  ObjectDetector();

  /**
   * @brief Detect red objects in the provided image
   *
   * @param msg ROS image message containing the input image
   * @return DetectionResult Structure containing detection results and debug
   * information
   */
  DetectionResult detect_red_object(
      const sensor_msgs::msg::Image::SharedPtr msg);

 private:
  /// Lower HSV color threshold for red object detection
  const cv::Scalar COLOR_LOWER_LIMIT{170, 50, 50};

  /// Upper HSV color threshold for red object detection
  const cv::Scalar COLOR_UPPER_LIMIT{255, 200, 90};

  /// Minimum area threshold for valid detection
  const double DETECTION_AREA_THRESHOLD = 0.0;

  /// Focal length of the camera in pixels
  const double FOCAL_LENGTH = 525.0;

  /// Known width of the target object in meters
  const double KNOWN_WIDTH = 0.3;

  /// Kernel used for morphological operations
  const cv::Mat MORPH_KERNEL =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

  /**
   * @brief Estimate distance to detected object using perspective projection
   *
   * @param pixel_width Width of the detected object in pixels
   * @return double Estimated distance to object in meters
   */
  double estimate_distance(double pixel_width) const;

  /**
   * @brief Apply morphological operations to clean up the binary mask
   *
   * Applies opening and closing operations to remove noise and fill holes
   * in the detection mask.
   *
   * @param mask Binary mask to be processed
   */
  void apply_morphological_operations(cv::Mat& mask) const;
};

#endif  // WALKER_OBJECT_DETECTOR_HPP
