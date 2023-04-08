# ompl_example_2d

This code works on Ros2 Humble. Check other branches of this repository to download the code for other ROS versions.

# Dependencies:

$ sudo apt-get install ros-humble-nav2-map-server libompl-dev ros-humble-moveit-planners-ompl

# Installation

$ cd ~/ros2_ws/src

$ git clone https://github.com/dominikbelter/ompl_example_2d

$ cd ~/ros2_ws/

$ colcon build --symlink-install

$ . install/setup.bash

# Run example

$ ros2 launch ompl_example_2d ompl_example_2d.launch.py

# Tasks

1. Use subscribed occupancy map to define forbiden states for the robot
2. Compare RRT-Connect algorithm with othes available planners
