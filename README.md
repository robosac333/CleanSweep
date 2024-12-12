# CleanSweep
This the final project of ENPM700: Software Development for Robotics

![CICD Workflow status](https://github.com/robosac333/CleanSweep/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/robosac333/CleanSweep/branch/main/graph/badge.svg?token=CODECOV_TOKEN)](https://codecov.io/gh/robosac333/CleanSweep) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Introducing CleanSweep, an autonomous warehouse cleanup robot developed for Acme Robotics to enhance workplace safety and efficiency. CleanSweep autonomously identifies, collects, and disposes of debris in dynamic warehouse environments, addressing the persistent challenge of maintaining clean, hazard-free workspaces. This innovative solution combines advanced navigation capabilities with precise object detection to ensure continuous workspace tidiness without human intervention.

CleanSweep achieves its functionality through two primary systems: an obstacle avoidance system and a perception-based debris collection system. The navigation system utilizes LiDAR-based obstacle detection for safe movement through complex warehouse layouts, while the perception system employs camera-based object detection to identify and locate debris. The robot operates in continuous cycles to collecting items. 

The platform is being built on ROS2 Humble and leverages Gazebo for simulation, ensuring robust performance in real-world applications. CleanSweep's efficient architecture allows it to seamlessly switch between obstacle avoidance and collection tasks while maintaining optimal performance. The system utilizes OpenCV for object detection and ROS2 sensor packages for obstacle avoidance, making it a versatile solution for warehouse maintenance. Detailed instructions for deploying and testing the system in a simulated environment are provided in subsequent sections.

## About the Authors
CleanSweep was developed by Sachin Jadhav and Navdeep Singh, both robotics graduate students at the University of Maryland.

Sachin, hailing from Parli, India,is a current Robotics Graduate student at University of Maryland, College Park. He holds a bachelor’s in mechanical engineering with a minor in computer science from SPCE, Mumbai. He has worked as a Robotics ML Engineer and is interested in leveraging AI and machine learning for perception and path planning in robotics, particularly using computer vision and reinforcement learning.

Navdeep, is from Delhi, India, graduated with a bachelor’s degree in Electronics and Communication Engineering from GGSIPU, Delhi. His interests lie in Deep Learning and Machine Learning, with experience in deploying ML models as a Perception Engineer at iTuple Technologies.

### AIP Workflow
This project is being developed using the Agile Iterative Process (AIP) alongside pair programming (driver and navigator roles), with a strong focus on test-driven development (TDD). The [product backlog](https://docs.google.com/spreadsheets/d/1ghUaM4df8IqF__fV75jPtsduJesjbfTFdYCTwERmi4M/edit?usp=sharing), iteration backlogs, and work log document each task completed up untill now. Each sprint is tagged for clarity, and detailed sprint planning and review meeting notes are available here, outlining our approach to building CleanSweep effectively.

Please find the links to the Sprint Planning and Review for our AIP Process [here](https://docs.google.com/document/d/1smmEauEUY9VQ0nROMU_jqvBjgAJWpHHJciYz5ddqwMs/edit?pli=1&tab=t.0).

## Video Demonstration
Please go to this [link](https://drive.google.com/drive/folders/1q-22uMjn5MP29nAcab462gSa1asZPphM?usp=sharing) for watching a video demonstration of the project.

## Features

- State-based robot control architecture
- Real-time obstacle detection and avoidance using LIDAR
- Computer vision-based red object detection using OpenCV
- Autonomous navigation and object approach behaviors
- Simulated environment with randomly placed objects
- Comprehensive unit testing suite

## Prerequisites

- ROS2 (Humble)
- Gazebo Simulator
- TurtleBot3 Packages
- OpenCV
- C++17 or later
- CMake
- Google Test Framework

## Installation

1. Create a ROS2 workspace (if not already created):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the repository:
```bash
git clone <repository-url> cleansweep
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the package:
```bash
colcon build --packages-select cleansweep

##Building for coverage
colcon build --cmake-args -DWANT_COVERAGE=ON
```

5. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Launch the simulation environment with the robot and randomly placed coke cans:
```bash
ros2 launch cleansweep walker_launch.py
```

This will:
- Start Gazebo with an empty warehouse environment
- Spawn a TurtleBot3 Waffle Pi robot
- Spawn multiple coke cans at random positions
- Launch the robot control node

## System Architecture

### Components

1. **Walker Node (`walker_bot.hpp`, `walker_bot.cpp`)**
   - Main robot control node
   - Implements state-based behavior system
   - Manages robot movement and object tracking
   - States include: Forward, Rotation, Alignment, and Approach

2. **Object Detector (`object_detector.hpp`, `object_detector.cpp`)**
   - Handles computer vision processing
   - Implements red object detection using HSV color space
   - Provides distance estimation to detected objects
   - Includes visualization for debugging

### State Machine

The robot operates in four primary states:
- **Forward State**: Default movement state with obstacle detection
- **Rotation State**: Activated when obstacles are detected
- **Alignment State**: Activated when a red object is detected
- **Approach State**: Manages controlled approach to detected objects

## Testing

The package includes a comprehensive testing suite using Google Test. To run the tests:

```bash
colcon test --packages-select cleansweep --event-handlers console_direct+
```

Test coverage includes:
- Object detection functionality
- Robot state transitions
- Obstacle detection and avoidance
- Movement control
- Sensor data processing

## Configuration

Key parameters can be adjusted in the header files:

### Object Detection (`object_detector.hpp`)
- HSV color thresholds for red object detection
- Detection area threshold
- Camera focal length and known object width
- Morphological operation parameters

### Robot Control (`walker_bot.hpp`)
- Safe distance threshold
- Target approach distance
- Alignment threshold
- Maximum angular speed

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## Acknowledgments

- Built using ROS2 and TurtleBot3 platform
- Uses OpenCV for computer vision processing
- Developed with Gazebo simulation environment

## Project Structure
```sh
src
└── cleansweep
       ├── CMakeLists.txt
       ├── include
       │   └── cleansweep
       │       ├── object_detector.hpp
       │       └── walker_bot.hpp
       ├── launch
       │   └── walker_launch.py
       ├── models
       │   └── coke_can
       │       └── model.sdf
       ├── package.xml
       ├── src
       │   ├── main.cpp
       │   ├── object_detector.cpp
       │   └── walker_bot.cpp
       ├── test
       │   ├── main.cpp
       │   ├── object_detection_test.cpp
       │   └── walker_bot_test.cpp
       └── worlds
           └── empty_warehouse.world
```

## Phase 0
Phase 0 focuses on the project proposal and provides information about the Agile Iterative Process (AIP) model to be used for software development throughout the project.

The Phase 0 project proposal, located [here](https://docs.google.com/document/d/1Ns__dJgFWKeA-ZFtLBhoYQTPA_1JrZVMxj8JlWUA-ok/edit?usp=sharing) , outlines a comprehensive plan and vision for the project, detailing the implementation strategies. It covers all the elements of a standard software development plan, offering an in-depth analysis.
 
