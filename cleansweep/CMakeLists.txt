cmake_minimum_required(VERSION 3.8)
project(cleansweep)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# Add coverage options
option(WANT_COVERAGE "Enable coverage reporting" ON)
if(WANT_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage -fprofile-arcs -ftest-coverage)
endif()

# Include the CodeCoverage module
include(${CMAKE_SOURCE_DIR}/cmake-modules/CodeCoverage.cmake)

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

# Create library targets
add_library(walker_bot
  src/walker_bot.cpp
)

add_library(object_detector
  src/object_detector.cpp
)

# Set include directories for libraries
target_include_directories(walker_bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(object_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Link dependencies to libraries
ament_target_dependencies(walker_bot
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Create executable
add_executable(robot_control src/main.cpp)

target_link_libraries(robot_control
  walker_bot
  object_detector
)

ament_target_dependencies(robot_control
  rclcpp
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Object detection test
  ament_add_gtest(object_detection_test
    test/object_detection_test.cpp
    test/main.cpp
  )
  target_link_libraries(object_detection_test 
    object_detector
    ${OpenCV_LIBRARIES}
  )
  
  # Walker bot test
  ament_add_gtest(walker_bot_test
    test/walker_bot_test.cpp
    test/main.cpp
  )
  target_link_libraries(walker_bot_test
    walker_bot
  )
endif()

# Install targets
install(TARGETS
  robot_control
  walker_bot
  object_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(WANT_COVERAGE)
  # Custom target for lcov
  add_custom_target(coverage
    COMMAND lcov --directory . --capture --output-file coverage.info
    COMMAND lcov --remove coverage.info '/usr/*' '*/test/*' --output-file coverage.info
    COMMAND lcov --list coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating coverage report"
  )

  # Custom target for gcovr HTML report
  add_custom_target(coverage_html
    COMMAND gcovr -r ${CMAKE_SOURCE_DIR} --html --html-details -o coverage.html
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating HTML coverage report"
  )
endif()

ament_package()