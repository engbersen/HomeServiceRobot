#!/bin/sh
catkin_dir=/home/workspace/catkin_ws
catkin_src=$catkin_dir/src

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src/World/my_world.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$catkin_src/World/map.yaml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " rosrun pick_objects pick_objects_node"