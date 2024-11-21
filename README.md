# CleanSweep
This the final project of ENPM700: Software Development for Robotics

![CICD Workflow status](https://github.com/robosac333/CleanSweep/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/robosac333/CleanSweep/branch/main/graph/badge.svg)](https://codecov.io/gh/robosac333/CleanSweep) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Introducing HALO (Human Awareness and Localization Optimizer), a perception system designed for Acme Robotics’ humanoid robot group. HALO enables humanoid robots to track and localize human movement around them, enhancing their awareness and helping them navigate spaces shared with people. This technology equips robots to understand their surroundings and provides real-time spatial data to optimize their path planning for effective, safe interactions.

HALO achieves its capabilities through two core components: an object detector and an object tracker. The object detector employs a YOLO v3 model to identify humans within the robot camera’s field of view (FOV), while the object tracker uses detections to calculate the relative position of people in relation to the robot’s camera using advanced geometric computer vision techniques. For accurate positioning, HALO assumes a fixed, known height of the robot’s camera relative to the ground.

HALO operates efficiently, delivering high performance on real-time human tracking and localization even with limited processing power, such as a laptop webcam on a CPU. The system’s precision makes it ideal for humanoid robots interacting in dynamic environments. Instructions for running the project’s demo, enabling similar performance on your own system, are provided in later sections.

About the Authors
HALO was developed by Sachin Jadhav and Navdeep Singh, both robotics graduate students at the University of Maryland.

Sachin, hailing from Parli, India, holds a bachelor’s and a master’s degree in mechanical engineering with a minor in computer science from SPCE, Mumbai. He has worked as a Robotics ML Engineer and is interested in leveraging AI and machine learning for perception and path planning in robotics, particularly using computer vision and reinforcement learning.

Navdeep, from Delhi, India, graduated with a bachelor’s degree in Electronics and Communication Engineering from GGSIPU, Delhi. His interests lie in Deep Learning and Machine Learning, with experience in deploying ML models as a Perception Engineer at Eye Tuple Technologies.

### AIP Workflow
This project was developed using the Agile Iterative Process (AIP) alongside pair programming (driver and navigator roles), with a strong focus on test-driven development (TDD). The [product backlog](https://docs.google.com/spreadsheets/d/1ghUaM4df8IqF__fV75jPtsduJesjbfTFdYCTwERmi4M/edit?gid=0#gid=0), iteration backlogs, and work log document each task completed in developing HALO. Each sprint is tagged for clarity, and detailed sprint planning and review meeting notes are available here, outlining our approach to building HALO effectively.

The latest (Phase 2) UML class and activity diagrams are in the Documentation/Phase_2 directory, while earlier versions for Phases 0 and 1 can be found in the Documentation/Phase0_Initial and Documentation/Phase1 directories.

A video providing a project overview and an explanation of the AIP process can be found below, or accessed directly

## Step2: Building the Workspace
After cloning this repository build the same
```bash
# Configure the project and generate a native build system:
  # Must re-run this command whenever any CMakeLists.txt file has been changed.
  cmake -S ./ -B build/
# Compile and build the project:
  # rebuild only files that are modified since the last build
  cmake --build build/
  # or rebuild everything from scracth
  cmake --build build/ --clean-first
  # to see verbose output, do:
  cmake --build build/ --verbose
# Run program:
  ./build/app/shell-app
# Run tests:
  cd build/; ctest; cd -
  # or if you have newer cmake
  ctest --test-dir build/
# Build docs:
  cmake --build build/ --target docs
  # open a web browser to browse the doc
  open docs/html/index.html
# Clean
  cmake --build build/ --target clean
# Clean and start over:
  rm -rf build/
```

### Building for Test Coverage
This section explains how to set up your code for test coverage reporting. You can view this information in the code coverage report by clicking the codecov badge at the top of this file. Alternatively, you can manually generate the report and view the HTML file in a web browser by executing the following commands:

```sh
# If you don't have gcovr or lcov installed, run:
  sudo apt-get install gcovr lcov
# Set the build type to Debug and enable coverage:
  cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Perform a clean compile, run unit tests, and generate the coverage report:
  cmake --build build/ --clean-first --target all test_coverage
# Open a web browser to view the test coverage report:
  open build/test_coverage/index.html
```
Running these commands will create an index.html page in the build/test_coverage sub-directory, which you can view in your local web browser.

You can also generate a code coverage report for the shell-app target instead of the unit tests. To do this, repeat the previous two steps, but target app_coverage:

```sh
# Clean compile, run the shell-app, and generate its coverage report:
  cmake --build build/ --clean-first --target all app_coverage
# Open a web browser to view the shell-app coverage report:
  open build/app_coverage/index.html
```

This will produce an index.html page in the build/app_coverage sub-directory that can be viewed in a local web browser.

### Google Coding Style

To check the conformity of your code to the Google C++ style guide, examine the results/cpplint_output.txt and results/cpplint_output.png files. These files will show the output from running the Cpplint tool on this project. Ideally, there should be no issues or problems, and all files should be processed successfully.

You can also perform a self-check by executing the following command from the highest-level directory of the project:

```sh
# Install Cpplint (ignore this step if it's already installed):
  sudo apt install cpplint
# Check code style conformity using Cpplint:
  cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```

Running the above command should yield the same output as found in the results/cpplint_output.txt file.

### Static Code Analysis

To perform static code analysis on this project, refer to the results/cppcheck_output.txt and results/cppcheck_output.png files to see the output from the Cppcheck tool. You should not encounter any issues, and all files should be checked successfully.

You can also verify this by running the following command in the highest-level directory of the project:

```sh
# Install Cppcheck (ignore this step if it's already installed):
  sudo apt install cppcheck
# Perform static code analysis using Cppcheck:
  cppcheck --enable=all --std=c++11 --suppress=missingInclude --suppress=unusedFunction $( find . -name *.cpp | grep -vE -e "^./build/" )
```

Executing this command should produce the same output as that found in the results/cppcheck_output.txt file.

HALO relies on the OpenCV library for computer vision functionalities and tools [details available here](https://github.com/opencv/opencv). It also employs a YOLO v3 model (originally developed by Joseph Redmon; more information [here](https://pjreddie.com/darknet/yolo/)). This deep-learning-based object detection model enables HALO to identify humans within a video feed, from which it then performs tracking. The YOLO v3 model is integrated from a PyTorch YOLO v3 configuration.

OpenCV and YOLO form the foundation of HALO’s tracking mechanism. OpenCV facilitates critical image processing tasks, while YOLO v3, as a benchmark object detection model, enables HALO to capture accurate human coordinates efficiently, offering flexible and reliable detection outputs. HALO’s architecture makes it versatile for object tracking, especially for humanoid robots requiring awareness in complex environments.

Please find the links to the video [here](https://youtu.be/GZoQa9Iq5oc?feature=shared) and the Sprint Planning and Review for our AIP Process [here](https://docs.google.com/document/d/1Bdwlqd9L5AySK-WD6bGSKijK-vkx62e66hNNi99mb8s/edit?pli=1&tab=t.0). The updated Product Backlog for the Phase 2 can be found [here](https://docs.google.com/spreadsheets/d/1ghUaM4df8IqF__fV75jPtsduJesjbfTFdYCTwERmi4M/edit?usp=sharing)

## Phase 1
In Phase 1 of the project, we set up the directory structure, create class headers and implementation. This phase also includes configuring GitHub Continuous Integration (CI), setting up Code Coverage with Codecov, and adding an MIT license along with their respective badges. All tasks will be completed using pair programming with a test-driven development (TDD) approach, following the Agile Iterative Process (AIP) software development model.

- The video for this Phase implementation can be found [here](https://youtu.be/I4-NO7tivLg?si=6dGpwYZuMBzqrTG6)

- Also please take a look at the link for the product back log [here](https://docs.google.com/spreadsheets/d/1ghUaM4df8IqF__fV75jPtsduJesjbfTFdYCTwERmi4M/edit?gid=0#gid=0)

## Phase 0
Phase 0 focuses on the project proposal and provides information about the Agile Iterative Process (AIP) model to be used for software development throughout the project.

The Phase 0 project report, located [here](https://docs.google.com/document/d/16jI77T4_1Mh9JJJpJmIrVLOkXnPECacXB7AYQt7YNdw/edit?usp=sharing) , outlines a comprehensive plan and vision for the project, detailing the implementation strategies. It covers all the elements of a standard software development plan, offering an in-depth analysis.

The quad chart, found at Documentation/Phase0_Initial/QuadChart.pdf, summarizes key aspects of the project, including the product, stakeholders, capabilities, and success criteria. This chart is a crucial component of the AIP methodology, which will guide the project’s progression.

Additionally, a brief video is embedded below, providing an overview of the project, including details from the report and quad chart. To view the video directly, click on the image below 

[![Video](https://i9.ytimg.com/vi_webp/ejj0KuOoPAQ/mqdefault.webp?v=67226555&sqp=COTJibkG&rs=AOn4CLDclj5uctrSXVySHKOsw5LqHJt-dQ)](https://youtu.be/ejj0KuOoPAQ?feature=shared)

### References
- [YOLO V3 weights model source](https://www.kaggle.com/datasets/shivam316/yolov3-weights)
- [YOLO v3 class names list](https://github.com/pjreddie/darknet/blob/master/data/coco.names)
- [OpenCV Installation](https://github.com/opencv/opencv)
