#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "cleansweep/object_detector.hpp"
#include <opencv2/opencv.hpp>

class ObjectDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
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

  cv::Mat createTestMat(int rows, int cols, const cv::Scalar& color) {
    return cv::Mat(rows, cols, CV_8UC3, color);
  }

  cv::Mat createRedTestObject(int rows, int cols, const cv::Rect& redRect) {
    cv::Mat image = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat modified = image.clone();
    modified(redRect) = cv::Scalar(0, 0, 255); // Pure red in BGR
    return modified;
  }

  std::unique_ptr<ObjectDetector> detector;
};

TEST_F(ObjectDetectorTest, InitializationTest) {
  ASSERT_NE(detector, nullptr);
}

TEST_F(ObjectDetectorTest, NoDetectionOnBlackImage) {
  cv::Mat black_image = createTestMat(480, 640, cv::Scalar(0, 0, 0));
  auto image_msg = createTestImage(black_image);
  auto result = detector->detect_red_object(image_msg);
  EXPECT_FALSE(result.detected);
}

TEST_F(ObjectDetectorTest, NoDetectionOnWhiteImage) {
  cv::Mat white_image = createTestMat(480, 640, cv::Scalar(255, 255, 255));
  auto image_msg = createTestImage(white_image);
  auto result = detector->detect_red_object(image_msg);
  EXPECT_FALSE(result.detected);
}

TEST_F(ObjectDetectorTest, DetectsRedObject) {
  cv::Mat test_image = createRedTestObject(480, 640, cv::Rect(270, 190, 100, 100));
  auto image_msg = createTestImage(test_image);
  auto detection = detector->detect_red_object(image_msg);
  
  EXPECT_TRUE(detection.detected);
  EXPECT_NEAR(detection.center.x, 320, 15);
  EXPECT_NEAR(detection.center.y, 240, 15);
}

TEST_F(ObjectDetectorTest, HandlesInvalidImageData) {
  auto invalid_msg = std::make_shared<sensor_msgs::msg::Image>();
  invalid_msg->encoding = sensor_msgs::image_encodings::BGR8;
  invalid_msg->height = 10;
  invalid_msg->width = 10;
  invalid_msg->step = 30;
  invalid_msg->data.resize(10);
  
  auto result = detector->detect_red_object(invalid_msg);
  EXPECT_FALSE(result.detected);
}

TEST_F(ObjectDetectorTest, DetectsLargerObject) {
  cv::Mat test_image = createRedTestObject(480, 640, cv::Rect(300, 150, 150, 200));
  auto image_msg = createTestImage(test_image);
  auto detection = detector->detect_red_object(image_msg);
  
  EXPECT_TRUE(detection.detected);
  EXPECT_NEAR(detection.center.x, 375, 15);
  EXPECT_NEAR(detection.center.y, 250, 15);
}

TEST_F(ObjectDetectorTest, HandlesVariousImageSizes) {
  std::vector<std::pair<int, int>> sizes = {{320, 240}, {640, 480}};
  
  for (const auto& size : sizes) {
    cv::Mat test_image = createRedTestObject(
      size.second, 
      size.first, 
      cv::Rect(size.first/2 - 50, size.second/2 - 50, 100, 100)
    );
    auto image_msg = createTestImage(test_image);
    auto detection = detector->detect_red_object(image_msg);
    
    EXPECT_TRUE(detection.detected);
    EXPECT_NEAR(detection.center.x, size.first/2, 15);
    EXPECT_NEAR(detection.center.y, size.second/2, 15);
  }
}