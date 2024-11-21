// turtlebot_test.cpp
#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cleansweep/turtlebot.hpp"
#include "cleansweep/obstacle_avoidance.hpp"

class TurtleBotTest : public ::testing::Test {
 protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        turtlebot_ = std::make_shared<TurtleBot>();
        obstacle_avoidance_ = std::make_shared<ObstacleAvoidance>();
    }

    void TearDown() override {
        turtlebot_.reset();
        obstacle_avoidance_.reset();
    }

    std::shared_ptr<TurtleBot> turtlebot_;
    std::shared_ptr<ObstacleAvoidance> obstacle_avoidance_;
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
    EXPECT_NO_THROW(turtlebot_->updatePosition(*obstacle_avoidance_));
}