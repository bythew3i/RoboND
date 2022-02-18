#!/bin/sh
export ROBOT_INITIAL_POSE="-x 0 -y 0 -Y 1.57"
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/../../world/jevin.world" &
sleep 3
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find turtlebot_gazebo)/../../map/map.yaml" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects"