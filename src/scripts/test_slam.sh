#!/bin/sh
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch my_robot world.launch " &

sleep 10

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch my_robot mapping.launch " &

sleep 10

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_teleop keyboard_teleop.launch "

