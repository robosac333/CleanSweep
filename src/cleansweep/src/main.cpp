/**
 * @file main.cpp
 * @brief Entry point for the Walker node.
 *
 * This file initializes the ROS 2 system and starts the Walker node, which
 * implements a robot navigation system with obstacle avoidance and object
 * detection capabilities. The Walker node uses laser scan data for obstacle
 * detection and camera input for identifying specific objects in the
 * environment.
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
 *
 * @author Sachin Ramesh Jadhav
 * @date December 2024
 * @version 1.0
 */

#include "cleansweep/walker_bot.hpp"

/**
 * @brief Main function to start the Walker node.
 *
 * Initializes the ROS 2 framework, creates an instance of the Walker node,
 * and spins it until the application is shut down. The function handles the
 * lifecycle of the ROS 2 node and ensures proper cleanup on shutdown.
 *
 * The function performs the following steps:
 * 1. Initializes the ROS 2 communication framework
 * 2. Creates a shared instance of the Walker node
 * 3. Enters the ROS 2 event loop to process callbacks
 * 4. Performs cleanup on shutdown
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code of the application (0 for normal exit, non-zero for
 * errors). Returns 0 on successful execution and cleanup.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Walker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
