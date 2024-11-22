# CleanSweep
This the final project of ENPM700: Software Development for Robotics

![CICD Workflow status](https://github.com/robosac333/CleanSweep/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/robosac333/CleanSweep/branch/main/graph/badge.svg?token=CODECOV_TOKEN)](https://codecov.io/gh/robosac333/CleanSweep) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Introducing CleanSweep, an autonomous warehouse cleanup robot developed for Acme Robotics to enhance workplace safety and efficiency. CleanSweep autonomously identifies, collects, and disposes of debris in dynamic warehouse environments, addressing the persistent challenge of maintaining clean, hazard-free workspaces. This innovative solution combines advanced navigation capabilities with precise object detection to ensure continuous workspace tidiness without human intervention.

CleanSweep achieves its functionality through two primary systems: an obstacle avoidance system and a perception-based debris collection system. The navigation system utilizes LiDAR-based obstacle detection for safe movement through complex warehouse layouts, while the perception system employs camera-based object detection to identify and locate debris. The robot operates in continuous cycles to collecting items. 

The platform is being built on ROS2 Humble and leverages Gazebo for simulation, ensuring robust performance in real-world applications. CleanSweep's efficient architecture allows it to seamlessly switch between obstacle avoidance and collection tasks while maintaining optimal performance. The system utilizes OpenCV for object detection and ROS2 sensor packages for obstacle avoidance, making it a versatile solution for warehouse maintenance. Detailed instructions for deploying and testing the system in a simulated environment are provided in subsequent sections.

About the Authors
CleanSweep was developed by Sachin Jadhav and Navdeep Singh, both robotics graduate students at the University of Maryland.

Sachin, hailing from Parli, India,is a current Robotics Graduate student at University of Maryland, College Park. He holds a bachelor’s in mechanical engineering with a minor in computer science from SPCE, Mumbai. He has worked as a Robotics ML Engineer and is interested in leveraging AI and machine learning for perception and path planning in robotics, particularly using computer vision and reinforcement learning.

Navdeep, is from Delhi, India, graduated with a bachelor’s degree in Electronics and Communication Engineering from GGSIPU, Delhi. His interests lie in Deep Learning and Machine Learning, with experience in deploying ML models as a Perception Engineer at iTuple Technologies.

### AIP Workflow
This project is being developed using the Agile Iterative Process (AIP) alongside pair programming (driver and navigator roles), with a strong focus on test-driven development (TDD). The [product backlog](https://docs.google.com/spreadsheets/d/1ghUaM4df8IqF__fV75jPtsduJesjbfTFdYCTwERmi4M/edit?usp=sharing), iteration backlogs, and work log document each task completed up untill now. Each sprint is tagged for clarity, and detailed sprint planning and review meeting notes are available here, outlining our approach to building CleanSweep effectively.


## Step 2: Building the Workspace
After cloning this repository in the src folder of your workspace:

```bash
# Navigate to the workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select cleansweep

# Source the setup files
source install/setup.bash
```

### Building for Test Coverage
This section explains how to set up your code for test coverage reporting. You can view this information in the code coverage report by clicking the codecov badge at the top of this file. Alternatively, you can manually generate the report and view the HTML file in a web browser by executing the following commands:

```sh
# If you don't have gcovr or lcov installed, run:
  sudo apt-get install lcov gcovr doxygen graphviz
# Run the tests
colcon test --packages-select cleansweep --event-handlers console_direct+

# View test results
colcon test-result --verbose


# Generate LCOV coverage report
cd build/cleansweep
make coverage
# The coverage report will be in coverage.info

# Generate HTML coverage report
make coverage_html
# The HTML report will be in coverage.html

# You can view the HTML coverage report by opening build/cleansweep/coverage.html in your web browser:
xdg-open build/cleansweep/coverage.html 
```
## Sprint Planning Sheet
Please find the links to the Sprint Planning and Review for our AIP Process [here](https://docs.google.com/document/d/1smmEauEUY9VQ0nROMU_jqvBjgAJWpHHJciYz5ddqwMs/edit?pli=1&tab=t.0).

## Phase 0
Phase 0 focuses on the project proposal and provides information about the Agile Iterative Process (AIP) model to be used for software development throughout the project.

The Phase 0 project proposal, located [here](https://docs.google.com/document/d/16jI77T4_1Mh9JJJpJmIrVLOkXnPECacXB7AYQt7YNdw/edit?usp=sharing) , outlines a comprehensive plan and vision for the project, detailing the implementation strategies. It covers all the elements of a standard software development plan, offering an in-depth analysis.
 
