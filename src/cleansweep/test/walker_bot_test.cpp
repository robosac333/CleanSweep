#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "cleansweep/walker_bot.hpp"
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