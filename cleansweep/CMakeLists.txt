cmake_minimum_required(VERSION 3.8)
project(cleansweep)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


# Print CV Bridge variables for debugging
# message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
# message(STATUS "cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")

# Include directories
include_directories(
  include
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Create the library targets
add_library(object_detection src/object_detection.cpp)

ament_target_dependencies(object_detection
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_include_directories(object_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(obstacle_avoidance src/obstacle_avoidance.cpp)

ament_target_dependencies(obstacle_avoidance
  rclcpp
  sensor_msgs
)

target_include_directories(obstacle_avoidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(turtlebot src/turtlebot.cpp)
ament_target_dependencies(turtlebot
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

target_include_directories(turtlebot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(turtlebot
  obstacle_avoidance
  object_detection
)
# Create the main executable
add_executable(cleansweep_node src/main.cpp)
target_link_libraries(cleansweep_node
  object_detection
  obstacle_avoidance
  turtlebot
)

ament_target_dependencies(cleansweep_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

# Install targets
install(TARGETS object_detection obstacle_avoidance turtlebot cleansweep_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(object_detection_test
    test/object_detection_test.cpp
    test/main.cpp
  )
  target_link_libraries(object_detection_test 
    object_detection
    # ${cv_bridge_LIBRARIES}
    # ${OpenCV_LIBRARIES}
  )
  
  ament_add_gtest(obstacle_avoidance_test
    test/obstacle_avoidance_test.cpp
    test/main.cpp
  )
  target_link_libraries(obstacle_avoidance_test obstacle_avoidance)
  
  ament_add_gtest(turtlebot_test
    test/turtlebot_test.cpp
    test/main.cpp
  )
  target_link_libraries(turtlebot_test turtlebot)
endif()

ament_package()
