/**
 * @file object_detector.cpp
 * @brief Implementation of the ObjectDetector class for detecting red objects
 * in images
 *
 * This file contains the implementation of methods for detecting and analyzing
 * red objects in images using OpenCV. It includes color-based detection,
 * morphological operations, and distance estimation.
 */

#include "cleansweep/object_detector.hpp"

/**
 * @brief Construct a new Object Detector object
 *
 * Initializes an ObjectDetector with default parameters for red object
 * detection.
 */
ObjectDetector::ObjectDetector() {}

/**
 * @brief Estimate the distance to an object based on its apparent width in
 * pixels
 *
 * Uses the principle of similar triangles to estimate distance based on the
 * known width of the target object and the camera's focal length.
 *
 * @param pixel_width Width of the object in pixels
 * @return double Estimated distance to the object in meters
 */
double ObjectDetector::estimate_distance(double pixel_width) const {
  return (KNOWN_WIDTH * FOCAL_LENGTH) / pixel_width;
}

/**
 * @brief Apply morphological operations to clean up the detection mask
 *
 * Performs closing followed by opening operations to remove noise and
 * fill small holes in the binary mask.
 *
 * @param mask Binary mask to be processed (modified in-place)
 */
void ObjectDetector::apply_morphological_operations(cv::Mat& mask) const {
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, MORPH_KERNEL);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, MORPH_KERNEL);
}

/**
 * @brief Detect red objects in the provided image
 *
 * Processes an input image to detect red objects using HSV color space
 * filtering. The method performs the following steps:
 * 1. Converts the image to HSV color space
 * 2. Creates a binary mask using color thresholds
 * 3. Applies morphological operations to clean the mask
 * 4. Finds contours and identifies the largest one
 * 5. Calculates the object's position and distance
 * 6. Creates visualization output
 *
 * @param msg ROS image message containing the input image
 * @return DetectionResult Structure containing detection results and debug
 * information:
 *         - detected: true if an object was found
 *         - distance: estimated distance to the object in meters
 *         - center: position of the object's center in the image
 *         - debug_mask: binary mask with detection visualization
 *         - debug_hsv: HSV converted image for debugging
 *
 * @throws cv_bridge::Exception if image conversion fails
 */
DetectionResult ObjectDetector::detect_red_object(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  DetectionResult result{false, 0.0, cv::Point2d(), cv::Mat(), cv::Mat()};

  try {
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Convert to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    result.debug_hsv = hsv_image.clone();

    // Create binary mask using color thresholds
    cv::Mat mask;
    cv::inRange(hsv_image, COLOR_LOWER_LIMIT, COLOR_UPPER_LIMIT, mask);

    // Clean up the mask using morphological operations
    apply_morphological_operations(mask);
    result.debug_mask = mask.clone();

    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      // Find the largest contour by area
      auto largest_contour =
          std::max_element(contours.begin(), contours.end(),
                           [](const std::vector<cv::Point>& c1,
                              const std::vector<cv::Point>& c2) {
                             return cv::contourArea(c1) < cv::contourArea(c2);
                           });

      double area = cv::contourArea(*largest_contour);

      // Check if the contour is large enough
      if (area > DETECTION_AREA_THRESHOLD) {
        cv::Rect bbox = cv::boundingRect(*largest_contour);

        // Calculate object center and distance
        result.center =
            cv::Point2d(bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);
        result.detected = true;
        result.distance = estimate_distance(bbox.width);

        // Add visualization elements to debug mask
        cv::rectangle(result.debug_mask, bbox, cv::Scalar(255), 2);
        cv::circle(result.debug_mask, cv::Point(result.center), 5,
                   cv::Scalar(255), -1);
      }
    }

    // Display debug visualization
    cv::namedWindow("Mask with Centroid", cv::WINDOW_NORMAL);
    cv::namedWindow("HSV Image", cv::WINDOW_NORMAL);
    cv::imshow("Mask with Centroid", result.debug_mask);
    cv::imshow("HSV Image", result.debug_hsv);
    cv::waitKey(1);

    return result;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ObjectDetector"),
                 "cv_bridge exception: %s", e.what());
    return result;
  }
}
