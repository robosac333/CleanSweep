#ifndef INCLUDE_CLEANSWEEP_OBJECT_DETECTION_HPP_
#define INCLUDE_CLEANSWEEP_OBJECT_DETECTION_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

class ObjectDetection : public rclcpp::Node {
 private:
  /// Node handle for ROS2 communication
  rclcpp::Node::SharedPtr nodeId;
  /// Image subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber;
  /// Image processing rate
  float processingRate;
  /// Source and filtered images
  cv::Mat sourceImage;
  cv::Mat filterImage;
  /// Target region in the image
  cv::Rect targetRegion;
  /// Image dimensions
  cv::Size dimensions;
  /// Object detection status
  bool detectionStatus;
  /// Collection of detected points
  std::vector<std::vector<cv::Point>> pointCollection;
  /// Threshold values for detection
  const cv::Scalar maxThreshold;
  const cv::Scalar minThreshold;
  
 public:
  /// Processed image output
  cv::Mat processedImage;

  /**
   * @brief  Default constructor
   * @param  none
   * @return none
   */
  explicit ObjectDetection();

  /**
   * @brief  Destructor
   * @param  none
   * @return none
   */
  ~ObjectDetection();

  /**
   * @brief  Process incoming image data
   * @param  Image data pointer from camera
   * @return void
   */
  void processInput(const std::shared_ptr<sensor_msgs::msg::Image> imageData);

  /**
   * @brief  Find target object
   * @param  Input image
   * @return bool Target found status
   */
  bool findTarget(cv::Mat image);

  /**
   * @brief  Apply filter to image
   * @param  Input image
   * @return Filtered image
   */
  cv::Mat applyFilter(cv::Mat image);

  /**
   * @brief  Get target area from the image
   * @return Rectangle defining target area
   */
  cv::Rect getTargetArea();

  /**
   * @brief  Set target area in the image
   * @param  rect Rectangle defining target area
   * @return void
   */
  void setTargetArea(cv::Rect rect);

  /**
   * @brief  Get detection status
   * @param  none
   * @return bool Detection status
   */
  bool getDetectionStatus();

  /**
   * @brief  Set detection status
   * @param  status Detection status to set
   * @return void
   */
  void setDetectionStatus(bool status);
};

#endif  // INCLUDE_CLEANSWEEP_OBJECT_DETECTION_HPP_