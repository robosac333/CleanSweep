// // object_detection_test.cpp
// #include <gtest/gtest.h>
// #include <memory>
// #include "rclcpp/rclcpp.hpp"
// #include "cleansweep/object_detection.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "cv_bridge/cv_bridge.h"
// #include "opencv2/opencv.hpp"

// class ObjectDetectionTest : public ::testing::Test {
//  protected:
//     void SetUp() override {
//         if (!rclcpp::ok()) {
//             rclcpp::init(0, nullptr);
//         }
//         object_detection_ = std::make_shared<ObjectDetection>();
//     }

//     void TearDown() override {
//         object_detection_.reset();
//     }

//     std::shared_ptr<ObjectDetection> object_detection_;
// };

// TEST_F(ObjectDetectionTest, TestConstructor) {
//     EXPECT_NO_THROW(ObjectDetection());
// }

// TEST_F(ObjectDetectionTest, TestProcessInput) {
//     auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
//     EXPECT_NO_THROW(object_detection_->processInput(image_msg));
// }

// TEST_F(ObjectDetectionTest, TestFindTarget) {
//     cv::Mat test_image(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
//     EXPECT_FALSE(object_detection_->findTarget(test_image));
// }

// TEST_F(ObjectDetectionTest, TestApplyFilter) {
//     cv::Mat test_image(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
//     cv::Mat result = object_detection_->applyFilter(test_image);
//     EXPECT_TRUE(result.empty());
// }

// TEST_F(ObjectDetectionTest, TestGetTargetArea) {
//     cv::Rect result = object_detection_->getTargetArea();
//     EXPECT_EQ(result, cv::Rect());
// }

// TEST_F(ObjectDetectionTest, TestSetTargetArea) {
//     cv::Rect test_rect(0, 0, 100, 100);
//     EXPECT_NO_THROW(object_detection_->setTargetArea(test_rect));
// }

// TEST_F(ObjectDetectionTest, TestDetectionStatus) {
//     EXPECT_FALSE(object_detection_->getDetectionStatus());
//     object_detection_->setDetectionStatus(true);
//     EXPECT_FALSE(object_detection_->getDetectionStatus());  // Will be false in stub
// }
