#!/bin/sh
xterm -e " gazebo world_file:=World/my_world.world" &
sleep 5
xterm -e " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm -e " rosrun rviz rviz"