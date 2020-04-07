# ompl_example_2d

# Dependencies:

$ sudo apt-get install ros-melodic-map-server ros-melodic-dwa-local-planner libompl-dev

# Installation

$ cd ~/catkin_ws/src

$ git clone https://github.com/dominikbelter/ompl_example_2d

$ cd ~/catkin_ws/

$ catkin_make

# Run example

$ roslaunch ompl_example_2d ompl_example_2d.launch

# Tasks

1. Use subscribed occupancy map to define forbiden states for the robot
2. Compare RRT-Connect algorithm with othes available planners
