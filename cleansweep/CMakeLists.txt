cmake_minimum_required(VERSION 3.8)
project(cleansweep)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add coverage options
option(WANT_COVERAGE "Enable coverage reporting" ON)
if(WANT_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage -fprofile-arcs -ftest-coverage)
endif()

# Doxygen support
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_CLASS_DIAGRAMS YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_CLASS_GRAPH YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_PACKAGE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
  set(DOXYGEN_TEMPLATE_RELATIONS YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)
  set(DOXYGEN_MAX_DOT_GRAPH_DEPTH 0)
  set(DOXYGEN_DOT_TRANSPARENT YES)

  # Create Doxygen target
  doxygen_add_docs(docs
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
    COMMENT "Generate documentation"
  )
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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

if(WANT_COVERAGE)
  # Custom target for lcov
  add_custom_target(coverage
    COMMAND lcov --directory . --capture --output-file coverage.info
    COMMAND lcov --remove coverage.info '/usr/*' '*/test/*' --output-file coverage.info
    COMMAND lcov --list coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating coverage report"
  )

  #Ccustom target for gcovr HTML report
  add_custom_target(coverage_html
    COMMAND gcovr -r ${CMAKE_SOURCE_DIR} --html --html-details -o coverage.html
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating HTML coverage report"
  )
endif()
ament_package()
