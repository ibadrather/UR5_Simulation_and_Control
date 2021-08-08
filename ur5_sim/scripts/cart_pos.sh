#!/bin/bash

# Build the catkin_ws
cd $(pwd)/../../..; catkin_make

# Launch the nodes
xterm  -e "source devel/setup.bash; roslaunch ur5_sim ur5_gazebo_joint_position_control.launch" &

sleep 10

xterm  -e "source devel/setup.bash; rosrun ur5_sim cart_pos" 
