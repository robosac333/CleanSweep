#include "cleansweep/object_detector.hpp"

ObjectDetector::ObjectDetector() {}

double ObjectDetector::estimate_distance(double pixel_width) const {
  return (KNOWN_WIDTH * FOCAL_LENGTH) / pixel_width;
}

void ObjectDetector::apply_morphological_operations(cv::Mat& mask) const {
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, MORPH_KERNEL);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, MORPH_KERNEL);
}

DetectionResult ObjectDetector::detect_red_object(const sensor_msgs::msg::Image::SharedPtr msg) {
  DetectionResult result{false, 0.0, cv::Point2d(), cv::Mat(), cv::Mat()};
  
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    // Convert to HSV
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    result.debug_hsv = hsv_image.clone();

    // Create mask using the color ranges
    cv::Mat mask;
    cv::inRange(hsv_image, COLOR_LOWER_LIMIT, COLOR_UPPER_LIMIT, mask);

    // Apply morphological operations to clean up the mask
    apply_morphological_operations(mask);
    result.debug_mask = mask.clone();

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
      // Find the largest contour
      auto largest_contour = std::max_element(contours.begin(), contours.end(),
          [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
              return cv::contourArea(c1) < cv::contourArea(c2);
          });
      
      double area = cv::contourArea(*largest_contour);
      
      if (area > DETECTION_AREA_THRESHOLD) {
        cv::Rect bbox = cv::boundingRect(*largest_contour);
        
        // Calculate centroid using bounding box
        result.center = cv::Point2d(
            bbox.x + bbox.width / 2.0,
            bbox.y + bbox.height / 2.0
        );
        
        result.detected = true;
        result.distance = estimate_distance(bbox.width);

        // Draw bounding rectangle and centroid on mask
        cv::rectangle(result.debug_mask, bbox, cv::Scalar(255), 2);
        cv::circle(result.debug_mask, cv::Point(result.center), 5, cv::Scalar(255), -1);
      }
    }

    // Create visualization windows
    cv::namedWindow("Mask with Centroid", cv::WINDOW_NORMAL);
    cv::namedWindow("HSV Image", cv::WINDOW_NORMAL);
    
    // Display results
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