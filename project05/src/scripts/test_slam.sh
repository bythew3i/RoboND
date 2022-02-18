#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/../../world/jevin.world" &
sleep 3
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 3
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "