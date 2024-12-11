#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "cleansweep/object_detector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

class ObjectDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Disable OpenCV GUI warnings
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
    detector = std::make_unique<ObjectDetector>();
  }

  sensor_msgs::msg::Image::SharedPtr createTestImage(const cv::Mat& cv_image) {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->height = cv_image.rows;
    msg->width = cv_image.cols;
    msg->encoding = sensor_msgs::image_encodings::BGR8;
    msg->step = cv_image.cols * cv_image.elemSize();
    msg->is_bigendian = false;
    
    size_t size = cv_image.total() * cv_image.elemSize();
    msg->data.resize(size);
    memcpy(msg->data.data(), cv_image.data, size);
    
    return msg;
  }

  std::unique_ptr<ObjectDetector> detector;
};

TEST_F(ObjectDetectorTest, InitializationTest) {
  ASSERT_NE(detector, nullptr);
}

TEST_F(ObjectDetectorTest, NoDetectionOnBlackImage) {
  cv::Mat black_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto image_msg = createTestImage(black_image);
  auto result = detector->detect_red_object(image_msg);
  EXPECT_FALSE(result.detected);
}

TEST_F(ObjectDetectorTest, NoDetectionOnWhiteImage) {
  cv::Mat white_image(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  auto image_msg = createTestImage(white_image);
  auto result = detector->detect_red_object(image_msg);
  EXPECT_FALSE(result.detected);
}

TEST_F(ObjectDetectorTest, DetectsRedObject) {
  // Create a pure red square in HSV color space
  cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);
  
  // Draw a rectangle with pure red HSV values that match our detector thresholds
  cv::Rect red_rect(270, 190, 100, 100);
  cv::Mat roi = hsv_image(red_rect);
  roi = cv::Scalar(175, 150, 70);  // H=175 (red), S=150 (saturated), V=70 (medium bright)
  
  // Convert back to BGR for the message
  cv::cvtColor(hsv_image, test_image, cv::COLOR_HSV2BGR);
  
  auto image_msg = createTestImage(test_image);
  auto result = detector->detect_red_object(image_msg);
  
  EXPECT_TRUE(result.detected);
  EXPECT_NEAR(result.center.x, 320, 15);
  EXPECT_NEAR(result.center.y, 240, 15);
}

TEST_F(ObjectDetectorTest, HandlesInvalidImageData) {
  // Create an invalid image message but with valid encoding to avoid cv_bridge error
  auto invalid_msg = std::make_shared<sensor_msgs::msg::Image>();
  invalid_msg->encoding = sensor_msgs::image_encodings::BGR8;
  invalid_msg->height = 10;
  invalid_msg->width = 10;
  invalid_msg->step = 30;  // 10 pixels * 3 channels
  // Create some invalid data
  invalid_msg->data.resize(10);  // Intentionally wrong size
  
  auto result = detector->detect_red_object(invalid_msg);
  EXPECT_FALSE(result.detected);
}

TEST_F(ObjectDetectorTest, DetectsLargerObject) {
  cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);
  
  // Small rectangle
  cv::Rect small_rect(100, 100, 100, 100);
  cv::Mat small_roi = hsv_image(small_rect);
  small_roi = cv::Scalar(175, 150, 70);
  
  // Large rectangle
  cv::Rect large_rect(300, 150, 150, 200);
  cv::Mat large_roi = hsv_image(large_rect);
  large_roi = cv::Scalar(175, 150, 70);
  
  cv::cvtColor(hsv_image, test_image, cv::COLOR_HSV2BGR);
  
  auto image_msg = createTestImage(test_image);
  auto result = detector->detect_red_object(image_msg);
  
  EXPECT_TRUE(result.detected);
  EXPECT_NEAR(result.center.x, 375, 15);
  EXPECT_NEAR(result.center.y, 250, 15);
}
