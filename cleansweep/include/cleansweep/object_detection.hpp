#ifndef INCLUDE_OBJECT_DETECTION_OBJECT_DETECTION_H_
#define INCLUDE_OBJECT_DETECTION_OBJECT_DETECTION_H_

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
  ObjectDetection();

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
  void processInput(const sensor_msgs::msg::Image::ConstPtr& imageData);

  /**
   * @brief  Find and filter target object
   * @param  Input image
   * @return bool Target found status and filtered image
   */
  bool findTarget(cv::Mat image) + applyFilter(cv::Mat image);

  /**
   * @brief  Get target area from the image
   * @param  Rectangle defining target area
   * @return void
   */
  cv::Rect getTargetArea() + setTargetArea(cv::Rect);

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

#endif  // INCLUDE_OBJECT_DETECTION_OBJECT_DETECTION_H_