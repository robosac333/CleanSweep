// obstacle_avoidance_test.cpp
#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "obstacle_avoidance/obstacle_avoidance.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleAvoidanceTest : public ::testing::Test {
 protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        obstacle_avoidance_ = std::make_shared<ObstacleAvoidance>();
    }

    void TearDown() override {
        obstacle_avoidance_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<ObstacleAvoidance> obstacle_avoidance_;
};

TEST_F(ObstacleAvoidanceTest, TestDefaultConstructor) {
    EXPECT_NO_THROW(ObstacleAvoidance());
}

TEST_F(ObstacleAvoidanceTest, TestParameterizedConstructor) {
    EXPECT_NO_THROW(ObstacleAvoidance(1.0));
}

TEST_F(ObstacleAvoidanceTest, TestDetectObstacle) {
    EXPECT_FALSE(obstacle_avoidance_->detectObstacle());
}

TEST_F(ObstacleAvoidanceTest, TestObstacleStatus) {
    // Test initial status
    EXPECT_FALSE(obstacle_avoidance_->getObstacleStatus());
    
    // Test setting status
    EXPECT_NO_THROW(obstacle_avoidance_->setObstacleStatus(true));
    EXPECT_FALSE(obstacle_avoidance_->getObstacleStatus()); // Still false in stub
}

TEST_F(ObstacleAvoidanceTest, TestProcessSensorData) {
    auto laser_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    EXPECT_NO_THROW(obstacle_avoidance_->processSensorData(laser_scan));
}