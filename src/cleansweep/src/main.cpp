/**
 * @file main.cpp
 * @brief Entry point for the Walker node.
 *
 * This file initializes the ROS 2 system and starts the Walker node.
 *
 * @copyright Copyright 2024 Sachin Ramesh Jadhav
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cleansweep/walker_bot.hpp"

/**
 * @brief Main function to start the Walker node.
 *
 * Initializes the ROS 2 framework, creates an instance of the Walker node,
 * and spins it until the application is shut down.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code of the application.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Walker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

