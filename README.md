# Map My World!

In this project, I use RTAB-Map package from ROS to create a 2D occupancy grid and 3D octomap from the simulated environment in Gazebo. RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. Below is an animation showing 2D occupancy grid and 3D octomap being generated in real-time as the robot traverses in its environment. 

![alt_text](https://github.com/schandrachary/Robotics/blob/map_my_world/images/rtabmap.gif)


### Prerequisites
1. ROS-Kinetic, Gazebo on Linux
2. CMake and g++
3. Install `rtabmap-ros` package: `$ sudo apt-get install ros-kinetic-rtabmap-ros`

### Launch Instructions

1. Clone the repo inside of `catkin_ws` directory and initialize catkin workspace:
```
$ mkdir catkin_ws && cd catkin_ws
$ git clone https://github.com/schandrachary/Robotics/tree/map_my_world.git
$ catkin_make
```
2. Launch Gazebo world and RViz, spawn the robot in the environment:
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

2. Open another terminal and launch `teleop` node:
```
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

3. Open another terminal and launch mapping node:
```
$ source devel/setup.bash
$ roslaunch my_robot mapping.launch
```

4. Navigating the robot in the environment creates a map that will be stored in `/home/workspace/catkin_ws/src` as a database file. Kill the mapping node launch
`rtabmap-databaseviewer`
```
$ rtabmap-databaseviewer /home/workspace/catkin_ws/src/rtabmap.db
```
- Choose view->Constraint View
- Choose view-> Graph View
- Choose Edit->View 3D map
