#!/bin/bash
clear
cd ROS
catkin_make
sleep 2
gnome-terminal --working-directory=${PWD} -e "roscore"
sleep 2
gnome-terminal --working-directory=${PWD} -e "rosrun turtlesim turtlesim_node"
sleep 2
gnome-terminal --working-directory=${PWD} -e "rosrun turtlesim turtlesim_teleop"
