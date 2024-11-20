cmake_minimum_required(VERSION 3.8)
project(cleansweep)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

# Find required packages
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # replaces roscpp
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)  # replaces move_base_msgs
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(GTest REQUIRED)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create executables
add_executable(object_collection 
  src/turtlebot.cpp 
  src/obstacle_avoidance.cpp 
)

add_executable(detector 
  src/object_detection.cpp 
)

# Link libraries
ament_target_dependencies(object_collection
  rclcpp
  sensor_msgs
  geometry_msgs
  nav2_msgs
  cv_bridge
  image_transport
)


ament_target_dependencies(detector
  rclcpp
  sensor_msgs
  geometry_msgs
  nav2_msgs
  cv_bridge
  image_transport
)

target_link_libraries(object_collection
  ${OpenCV_LIBS}
)

target_link_libraries(detector
  ${OpenCV_LIBS}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  ament_add_gtest(test_cleansweep
    test/main.cpp
    test/obstacle_avoidance_test.cpp
    test/turtlebot_test.cpp
    test/object_detection_test.cpp
    src/obstacle_avoidance.cpp
    src/turtlebot.cpp
    src/object_detection.cpp
  )
  
  target_link_libraries(test_cleansweep
    ${PROJECT_NAME}
    GTest::GTest
    GTest::Main
    ${OpenCV_LIBS}
  )
  
  ament_target_dependencies(test_cleansweep
    rclcpp
    sensor_msgs
    geometry_msgs
    nav2_msgs
    cv_bridge
    image_transport
  )
endif()


# Install executables
install(TARGETS
  object_collection
  detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if you have any
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
