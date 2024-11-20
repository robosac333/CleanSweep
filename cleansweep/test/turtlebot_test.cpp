// turtlebot_test.cpp
#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "turtlebot/turtlebot.hpp"
#include "obstacle_avoidance/obstacle_avoidance.hpp"

class TurtleBotTest : public ::testing::Test {
 protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        turtlebot_ = std::make_shared<TurtleBot>();
    }

    void TearDown() override {
        turtlebot_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<TurtleBot> turtlebot_;
};

TEST_F(TurtleBotTest, TestDefaultConstructor) {
    EXPECT_NO_THROW(TurtleBot());
}

TEST_F(TurtleBotTest, TestParameterizedConstructor) {
    EXPECT_NO_THROW(TurtleBot(0.5, 0.2));
}

TEST_F(TurtleBotTest, TestSetForwardSpeed) {
    float speed = 0.5;
    EXPECT_EQ(turtlebot_->setForwardSpeed(speed), 0.0);
}

TEST_F(TurtleBotTest, TestSetRotationSpeed) {
    float speed = 0.2;
    EXPECT_EQ(turtlebot_->setRotationSpeed(speed), 0.0);
}

TEST_F(TurtleBotTest, TestVerifySpeedChange) {
    EXPECT_FALSE(turtlebot_->verifySpeedChange());
}

TEST_F(TurtleBotTest, TestResetPosition) {
    EXPECT_FALSE(turtlebot_->resetPosition());
}

TEST_F(TurtleBotTest, TestUpdatePosition) {
    ObstacleAvoidance obstacleAvoidance;
    EXPECT_NO_THROW(turtlebot_->updatePosition(obstacleAvoidance));
}