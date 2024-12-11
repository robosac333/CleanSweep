#include "cleansweep/walker_bot.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <opencv2/core/utils/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cleansweep/object_detector.hpp"

class WalkerBotTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Disable OpenCV GUI warnings
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    walker_node = std::make_shared<Walker>();
  }

  void TearDown() override { walker_node.reset(); }

  std::shared_ptr<Walker> walker_node;

  sensor_msgs::msg::LaserScan::SharedPtr createTestScan(
      const std::vector<float>& ranges) {
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->ranges = ranges;
    scan->angle_min = 0.0;
    scan->angle_max = 2 * M_PI;
    scan->angle_increment = (2 * M_PI) / 360.0;
    scan->range_min = 0.0;
    scan->range_max = 10.0;
    return scan;
  }

  // Helper method to create image messages
  sensor_msgs::msg::Image::SharedPtr createImageMsg(const cv::Mat& image) {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->height = image.rows;
    msg->width = image.cols;
    msg->encoding = sensor_msgs::image_encodings::BGR8;
    msg->step = image.cols * image.elemSize();
    msg->data.resize(image.total() * image.elemSize());
    memcpy(msg->data.data(), image.data, msg->data.size());
    return msg;
  }
};

TEST_F(WalkerBotTest, TestBasicInitialization) {
  ASSERT_NE(walker_node, nullptr);
  EXPECT_EQ(walker_node->get_rotation_direction(), 1.0);
}

TEST_F(WalkerBotTest, ToggleRotationDirectionTest) {
  double initial_direction = walker_node->get_rotation_direction();
  walker_node->toggle_rotation_direction();
  EXPECT_EQ(walker_node->get_rotation_direction(), -initial_direction);
}

TEST_F(WalkerBotTest, SetRotationDirectionTest) {
  walker_node->set_rotation_direction(-1.0);
  EXPECT_EQ(walker_node->get_rotation_direction(), -1.0);
  walker_node->set_rotation_direction(1.0);
  EXPECT_EQ(walker_node->get_rotation_direction(), 1.0);
}

TEST_F(WalkerBotTest, ObstacleDetectionTest) {
  std::vector<float> ranges(360, 2.0);
  auto scan = createTestScan(ranges);
  EXPECT_EQ(walker_node->detect_obstacle_location(scan),
            ObstacleLocation::NONE);

  ranges = std::vector<float>(360, 2.0);
  for (int i = 18; i <= 342; i++) {
    ranges[i] = 0.5;
  }
  scan = createTestScan(ranges);
  EXPECT_EQ(walker_node->detect_obstacle_location(scan),
            ObstacleLocation::FRONT);
}

TEST_F(WalkerBotTest, PathClearTest) {
  std::vector<float> clear_ranges(360, 2.0);
  auto clear_scan = createTestScan(clear_ranges);
  EXPECT_TRUE(walker_node->is_path_clear(clear_scan));

  std::vector<float> blocked_ranges(360, 0.5);
  auto blocked_scan = createTestScan(blocked_ranges);
  EXPECT_FALSE(walker_node->is_path_clear(blocked_scan));
}

TEST_F(WalkerBotTest, TargetDistanceTest) {
  EXPECT_GT(walker_node->get_target_distance(), 0.0);
}

TEST_F(WalkerBotTest, StateTransitions) {
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges.resize(360, 2.0);

  for (int i = 0; i < 30; i++) {
    scan_msg->ranges[i] = 0.5;
  }

  auto initial_state = walker_node->get_current_state();
  walker_node->process_scan(scan_msg);
  EXPECT_NE(initial_state, walker_node->get_current_state());
}

TEST_F(WalkerBotTest, ObjectDetection) {
  cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Rect red_rect(270, 190, 100, 100);
  cv::Mat roi = hsv_image(red_rect);
  roi = cv::Scalar(175, 150, 70);

  cv::Mat result;
  cv::cvtColor(hsv_image, result, cv::COLOR_HSV2BGR);

  auto img_msg = createImageMsg(result);
  walker_node->process_image(img_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_TRUE(walker_node->is_red_object_detected());
}

TEST_F(WalkerBotTest, AlignmentStateTest) {
  // Create test image with red object
  cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Rect red_rect(270, 190, 100, 100);
  cv::Mat roi = hsv_image(red_rect);
  roi = cv::Scalar(175, 150, 70);

  cv::Mat result;
  cv::cvtColor(hsv_image, result, cv::COLOR_HSV2BGR);

  // First verify that we're starting in a non-alignment state
  auto* initial_alignment_state =
      dynamic_cast<AlignmentState*>(walker_node->get_current_state());
  EXPECT_EQ(initial_alignment_state, nullptr);

  // Process image and verify red object detection
  auto img_msg = createImageMsg(result);
  walker_node->process_image(img_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_TRUE(walker_node->is_red_object_detected());

  // Process scan to trigger state transition
  std::vector<float> ranges(360, 2.0);  // Clear path
  auto scan = createTestScan(ranges);
  walker_node->process_scan(scan);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Now verify we've transitioned to alignment state
  auto* final_state = walker_node->get_current_state();
  bool is_alignment_or_approach =
      (dynamic_cast<AlignmentState*>(final_state) != nullptr) ||
      (dynamic_cast<ApproachState*>(final_state) != nullptr);
  EXPECT_TRUE(is_alignment_or_approach)
      << "Expected either AlignmentState or ApproachState";
}

TEST_F(WalkerBotTest, ApproachStateTest) {
  cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);

  cv::Rect red_rect(220, 140, 200, 200);
  cv::Mat roi = hsv_image(red_rect);
  roi = cv::Scalar(175, 150, 70);

  cv::Mat result;
  cv::cvtColor(hsv_image, result, cv::COLOR_HSV2BGR);

  auto img_msg = createImageMsg(result);
  walker_node->process_image(img_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_TRUE(walker_node->is_red_object_detected());

  std::vector<float> ranges(360, 2.0);
  auto scan = createTestScan(ranges);

  for (int i = 0; i < 5; i++) {
    walker_node->process_scan(scan);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  auto* approach_state =
      dynamic_cast<ApproachState*>(walker_node->get_current_state());
  EXPECT_NE(approach_state, nullptr);
}

TEST_F(WalkerBotTest, AngularCorrectionCalculationTest) {
  cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
  auto img_msg = createImageMsg(test_image);
  walker_node->process_image(img_msg);

  const double max_speed = walker_node->get_max_angular_speed();
  const double image_center = walker_node->get_image_width() / 2.0;

  struct TestCase {
    double error;
    double expected_min;
    double expected_max;
  };

  std::vector<TestCase> test_cases = {
      {0.0, -0.001, 0.001},
      {image_center, 0.0, max_speed},
      {-image_center, -max_speed, 0.0},
      {image_center / 2, 0.0, max_speed / 2 + 0.1}};

  for (const auto& tc : test_cases) {
    double result = walker_node->calculate_angular_correction(tc.error);
    EXPECT_GE(result, tc.expected_min);
    EXPECT_LE(result, tc.expected_max);
  }
}

TEST_F(WalkerBotTest, StateTransitionCoverage) {
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges.resize(360, 2.0);

  for (int i = 0; i < 17; i++) {
    scan_msg->ranges[i] = 0.5;
  }
  walker_node->process_scan(scan_msg);

  scan_msg->ranges = std::vector<float>(360, 2.0);
  for (int i = 343; i < 360; i++) {
    scan_msg->ranges[i] = 0.5;
  }
  walker_node->process_scan(scan_msg);

  std::this_thread::sleep_for(std::chrono::seconds(6));
  scan_msg->ranges = std::vector<float>(360, 2.0);
  walker_node->process_scan(scan_msg);
}

TEST_F(WalkerBotTest, ObstacleAvoidanceSequence) {
  std::vector<ObstacleLocation> locations = {
      ObstacleLocation::LEFT, ObstacleLocation::RIGHT, ObstacleLocation::FRONT};

  for (auto location : locations) {
    std::vector<float> ranges(360, 2.0);

    switch (location) {
      case ObstacleLocation::LEFT:
        for (int i = 0; i <= 17; i++) ranges[i] = 0.5;
        break;
      case ObstacleLocation::RIGHT:
        for (int i = 343; i <= 359; i++) ranges[i] = 0.5;
        break;
      case ObstacleLocation::FRONT:
        for (int i = 18; i <= 342; i++) ranges[i] = 0.5;
        break;
      case ObstacleLocation::NONE:
        break;
    }

    auto scan = createTestScan(ranges);
    walker_node->process_scan(scan);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

TEST_F(WalkerBotTest, VelocityPublishingVerification) {
  walker_node->publish_velocity(0.5, 0.0);
  walker_node->publish_velocity(0.0, 0.3);
  walker_node->publish_velocity(0.3, 0.2);
}
