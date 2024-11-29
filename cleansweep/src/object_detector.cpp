#include "cleansweep/object_detector.hpp"

ObjectDetector::ObjectDetector() {}

double ObjectDetector::estimate_distance(double pixel_width) const {
  return (KNOWN_WIDTH * FOCAL_LENGTH) / pixel_width;
}

DetectionResult ObjectDetector::detect_red_object(const sensor_msgs::msg::Image::SharedPtr msg) {
  DetectionResult result{false, 0.0, cv::Point2d()};
  
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Create mask using the coke can color ranges
    cv::Mat mask;
    cv::inRange(hsv_image, COLOR_LOWER_LIMIT, COLOR_UPPER_LIMIT, mask);

    // Get image size to modify size of mask
    cv::Size imageSize = cv_ptr->image.size();
    mask(cv::Rect(0, 0, imageSize.width, 0.8*imageSize.height)) = 0;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Create a copy of the original image for visualization
    cv::Mat display_image = cv_ptr->image.clone();

    // Draw all contours in blue
    cv::drawContours(display_image, contours, -1, cv::Scalar(255, 0, 0), 2);

    if (!contours.empty()) {
      // Find the largest contour
      auto largest_contour = std::max_element(contours.begin(), contours.end(),
          [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
              return cv::contourArea(c1) < cv::contourArea(c2);
          });
      
      double area = cv::contourArea(*largest_contour);
      
      if (area > DETECTION_AREA_THRESHOLD) {
        cv::Rect bbox = cv::boundingRect(*largest_contour);
        cv::Moments moments = cv::moments(*largest_contour);
        
        result.detected = true;
        result.center = cv::Point2d(moments.m10/moments.m00, moments.m01/moments.m00);
        result.distance = estimate_distance(bbox.width);

        // Draw the largest contour in green to distinguish it
        cv::drawContours(display_image, std::vector<std::vector<cv::Point>>{*largest_contour}, 
                        0, cv::Scalar(0, 255, 0), 2);

        // Draw the center point
        cv::circle(display_image, cv::Point(result.center), 5, cv::Scalar(0, 0, 255), -1);
        
        // Draw distance information
        std::string distance_text = "Distance: " + std::to_string(result.distance).substr(0, 4) + "m";
        cv::putText(display_image, distance_text, 
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, 
                    cv::Scalar(0, 255, 0), 2);

        mask(cv::Rect(0, 0, 0.3*imageSize.width, imageSize.height)) = 0;
      }
    }

    cv::namedWindow("HSV Image");
    cv::namedWindow("Turtlebot View");
    cv::imshow("HSV Image", hsv_image);
    cv::imshow("Turtlebot View", display_image);  // Show the image with visualizations
    cv::waitKey(1);

    return result;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ObjectDetector"), 
                 "cv_bridge exception: %s", e.what());
    return result;
  }
}