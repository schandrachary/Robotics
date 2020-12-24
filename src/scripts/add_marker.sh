#!/bin/sh
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch my_robot world.launch " &

sleep 10

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl.launch " &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch " &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosrun add_markers add_markers " &
