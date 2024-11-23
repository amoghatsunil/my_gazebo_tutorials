# Walker Robot

## Overview

This repository contains state pattern implementation for a walker robot to perform task including Moving Forward and Rotating based on laser scan sensor data for avoiding obstacles.

The module implements a simple walker algorithm similar to a Roomba robot vacuum cleaner. where the robot moves forward until it encounters an obstacle (without colliding), then it rotates in place until the path ahead is clear.  After that, it moves forward again and repeat the process.  Each time the robot rotates, it alternates between rotating clockwise and counterclockwise.

States implemented : MoveForward, Rotate

## Author

Amogha Thalihalla Sunil (amoghats@umd.edu)

## Prerequisites

- ROS2 Humble installed
- Colcon build tool
- C++17 compatible compiler

* Turtlebot3

**Link to Rosbag Recording** : https://drive.google.com/drive/folders/1dRSHmA2Y1sY4i9Rrje1xTHRswvwa1v0V?usp=sharing

## Installation and building the setup

1. **Create a ROS Workspace if not already created**

   ```sh
   cd 
   mkdir -p ros2_ws/src/
   cd ~/ros2_ws/src/ 
   ```
2. **Create a ROS package "walker"**

   ```sh
   ros2 pkg create --build-type ament_cmake --license Apache-2.0 walker
   ```
3. **Check for Missing Dependencies Before Building**

   To run rosdep in the root of the workspace:

   ```sh
   cd ~/ros2_ws 
   rosdep install -i --from-path src --rosdistro humble -y
   ```
4. **Build the package**
   Use colcon to build the package

   ```sh
   colcon build --packages-select walker
   ```
5. **Source the setup**

   Source the script setup to overlay this workspace on the environment

   ```sh
   source install/setup.bash
   ```

## Using the package / Running the Package

ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py**Running and bringing up the Turtlebot gazebo simulation**

1. To run the talker node, run the following command

   ```sh
   echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models/" >> ~/.bashrc
   echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
   source ~/.bashrc
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
   ```
2. **Running the walker node through launch file for rosbag optional recording**

   record_bag flag can be set to false if rosbag recording is not required!

   ```sh
   ros2 launch walker walker.launch.py record_bag:=true
   ```

## ROS2 Bag to Record and Replay

1. **Play the recording**:

   ```bash
   ros2 bag play results/rosbag
   ```

**Topics Used**

'/scan' from lidar for obstacle detection.

/cmd_vel' from odom to publish linear and angular velocity.

```
The repository structure should be as follows 

```plaintext
ros2_ws/
├── src/
│   ├── walker/
│       ├── include/
│       │   ├── walker/
│       │       ├── walker_node.hpp
│       ├── src/
│       │   ├── walker_node.cpp
│       ├── launch/
│       │   ├── walker.launch.py
│       ├── Results/
│       │   ├── rosbag(rosbagfiles)
│       │   ├── cpplint_output.txt
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── README.md
│       ├── LICENSE
```

## Linting

cpplint has been run and the output is saved in the ``cpplintOutput.txt`` file

To run cpplint run the following command :

```sh
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -v "/build/" )
```

## Clangd formatting

clangd formatting is checked and has no more changes required.
Screenshot is attached.

```sh
clang-format -style=Google -i src/walker/src/*.cpp
```

## License

This project is licensed under the BSD-3-Clause Licence. Check the license file for details

## Acknowledgements

- Open Source Robotics Foundation, Inc.
- ROS2 Community
