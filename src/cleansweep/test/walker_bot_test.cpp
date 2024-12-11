#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "cleansweep/walker_bot.hpp"
#include "cleansweep/object_detector.hpp"
#include <memory>

class WalkerBotTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    walker_node = std::make_shared<Walker>();
  }

  void TearDown() override {
    walker_node.reset();
  }

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
  EXPECT_EQ(walker_node->detect_obstacle_location(scan), ObstacleLocation::NONE);

  ranges = std::vector<float>(360, 2.0);
  for(int i = 18; i <= 342; i++) {
    ranges[i] = 0.5;
  }
  scan = createTestScan(ranges);
  EXPECT_EQ(walker_node->detect_obstacle_location(scan), ObstacleLocation::FRONT);
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
  // Test Forward to Rotation state transition
  auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_msg->ranges.resize(360, 2.0);  // Clear path
  
  // Simulate obstacle in front
  for(int i = 0; i < 30; i++) {
    scan_msg->ranges[i] = 0.5;  // Obstacle within SAFE_DISTANCE
  }
  
  // Get initial state
  auto initial_state = walker_node->get_current_state();
  
  // Process scan
  walker_node->process_scan(scan_msg);
  
  // Verify state changed
  EXPECT_NE(initial_state, walker_node->get_current_state());
}
TEST_F(WalkerBotTest, ObjectDetection) {
  // Create test image using OpenCV
  cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);
  
  // Draw a rectangle with pure red HSV values that match detector thresholds
  cv::Rect red_rect(270, 190, 100, 100);
  cv::Mat roi = hsv_image(red_rect);
  roi = cv::Scalar(175, 150, 70);  // H=175 (red), S=150 (saturated), V=70 (medium bright)
  
  // Convert back to BGR for the message
  cv::cvtColor(hsv_image, test_image, cv::COLOR_HSV2BGR);
  
  // Create ROS message from OpenCV image
  auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
  img_msg->height = test_image.rows;
  img_msg->width = test_image.cols;
  img_msg->encoding = sensor_msgs::image_encodings::BGR8;
  img_msg->step = test_image.cols * test_image.elemSize();
  img_msg->is_bigendian = false;
  
  size_t size = test_image.total() * test_image.elemSize();
  img_msg->data.resize(size);
  memcpy(img_msg->data.data(), test_image.data, size);

  // Process image
  walker_node->process_image(img_msg);
  
  // Add small delay to allow processing
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Print debug info
  RCLCPP_INFO(walker_node->get_logger(), 
              "Object detection test: detected=%d, distance=%.2f", 
              walker_node->is_red_object_detected(),
              walker_node->get_object_distance());
  
  // Verify object detected
  EXPECT_TRUE(walker_node->is_red_object_detected());
}

TEST_F(WalkerBotTest, AlignmentStateTest) {
  // Create a scan with clear path
  std::vector<float> ranges(360, 2.0);
  auto scan = createTestScan(ranges);
  
  // Setup red object detection
  cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);
  
  cv::Rect red_rect(270, 190, 100, 100);
  cv::Mat roi = hsv_image(red_rect);
  roi = cv::Scalar(175, 150, 70);
  cv::cvtColor(hsv_image, test_image, cv::COLOR_HSV2BGR);
  
  auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
  img_msg->height = test_image.rows;
  img_msg->width = test_image.cols;
  img_msg->encoding = sensor_msgs::image_encodings::BGR8;
  img_msg->step = test_image.cols * test_image.elemSize();
  img_msg->data.resize(test_image.total() * test_image.elemSize());
  memcpy(img_msg->data.data(), test_image.data, img_msg->data.size());

  // Process image to trigger alignment state
  walker_node->process_image(img_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify state changed to alignment
  EXPECT_TRUE(walker_node->is_red_object_detected());
  
  // Process scan in alignment state
  walker_node->process_scan(scan);
}

TEST_F(WalkerBotTest, ApproachStateTest) {
  // Create a scan with clear path
  std::vector<float> ranges(360, 2.0);
  auto scan = createTestScan(ranges);
  
  // Setup red object detection at close range
  cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat hsv_image;
  cv::cvtColor(test_image, hsv_image, cv::COLOR_BGR2HSV);
  
  // Create larger object to simulate closer distance
  cv::Rect red_rect(220, 140, 200, 200);
  cv::Mat roi = hsv_image(red_rect);
  roi = cv::Scalar(175, 150, 70);
  cv::cvtColor(hsv_image, test_image, cv::COLOR_HSV2BGR);
  
  auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
  img_msg->height = test_image.rows;
  img_msg->width = test_image.cols;
  img_msg->encoding = sensor_msgs::image_encodings::BGR8;
  img_msg->step = test_image.cols * test_image.elemSize();
  img_msg->data.resize(test_image.total() * test_image.elemSize());
  memcpy(img_msg->data.data(), test_image.data, img_msg->data.size());

  // Process image to trigger states
  walker_node->process_image(img_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Process multiple scans to allow state transitions
  for(int i = 0; i < 5; i++) {
    walker_node->process_scan(scan);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

TEST_F(WalkerBotTest, RotationStateWithTimerTest) {
  // Create scan with obstacle
  std::vector<float> ranges(360, 2.0);
  for(int i = 18; i <= 342; i++) {
    ranges[i] = 0.5;  // Obstacle in front
  }
  auto scan = createTestScan(ranges);

  // Process scan to enter rotation state
  walker_node->process_scan(scan);
  
  // Allow rotation timer to expire
  std::this_thread::sleep_for(std::chrono::seconds(6));
  
  // Process scan again after timer
  ranges = std::vector<float>(360, 2.0);  // Clear path
  auto clear_scan = createTestScan(ranges);
  walker_node->process_scan(clear_scan);
}

TEST_F(WalkerBotTest, AngularCorrectionCalculationTest) {
  // Initialize with a test image first
  cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
  img_msg->height = test_image.rows;
  img_msg->width = test_image.cols;
  img_msg->encoding = sensor_msgs::image_encodings::BGR8;
  img_msg->step = test_image.cols * test_image.elemSize();
  img_msg->data.resize(test_image.total() * test_image.elemSize());
  memcpy(img_msg->data.data(), test_image.data, img_msg->data.size());
  
  // Process image to initialize width
  walker_node->process_image(img_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Print debug info
  RCLCPP_INFO(walker_node->get_logger(), 
              "Image width initialized to: %d", 
              walker_node->get_image_width());
              
  // Verify image width is initialized
  ASSERT_GT(walker_node->get_image_width(), 0) << "Image width must be initialized before testing";
  
  const double max_speed = walker_node->get_max_angular_speed();
  const double image_center = walker_node->get_image_width() / 2.0;
  
  // Test cases with debug output
  struct TestCase {
    double error;
    double expected_min;
    double expected_max;
  };
  
  std::vector<TestCase> test_cases = {
    {0.0, -0.001, 0.001},                    // Center - should give ~0
    {image_center, 0.0, max_speed},          // Full right - should give positive < max
    {-image_center, -max_speed, 0.0},        // Full left - should give negative > -max
    {image_center/2, 0.0, max_speed/2 + 0.1} // Halfway right - should give ~half max
  };
  
  for (const auto& tc : test_cases) {
    double result = walker_node->calculate_angular_correction(tc.error);
    RCLCPP_INFO(walker_node->get_logger(), 
                "Testing error=%.2f, got correction=%.2f (expected between %.2f and %.2f)",
                tc.error, result, tc.expected_min, tc.expected_max);
                
    EXPECT_GE(result, tc.expected_min) 
        << "Angular correction too low for error=" << tc.error;
    EXPECT_LE(result, tc.expected_max) 
        << "Angular correction too high for error=" << tc.error;
  }
}