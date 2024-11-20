// main_test.cpp
#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"

// Include all test files
#include "object_detection/object_detection_test.cpp"
#include "obstacle_avoidance/obstacle_avoidance_test.cpp"
#include "turtlebot/turtlebot_test.cpp"

class TestRunner {
 public:
    static void SetUpTestCase() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestCase() {
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new TestRunner());
    return RUN_ALL_TESTS();
}