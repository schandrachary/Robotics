# Build My World
I call my robot Chakra. The purpose of this lab is build a four-wheeled robot model with the Model Editor in Gazeo. 
Also build a model to house this robot in Building Editor. Include this model in a world file, I call chakraWorld. And, finally write a plugin to interact with this world. 

### Directory Structure

```
    .myrobot                      # myrobot lab main folder
    |-- model
    |   |-- 4wheelRobot           # Model files of the four-wheeled robot
    |   |   |-- model.config
    |   |   |-- model.sdf
    |   |-- 4wheelRobot
    |   |   |-- Chakra-4          # Chakra-4 is the working version that is added to the world
    |   |   |   |-- model.config
    |   |   |   |-- model.sdf
    |-- script                    # Chakra World plugin C++ script
    |   |-- welcome_message.cpp
    |-- world                     # Chakra world with condo layout and furniture included
    |   |--- chakraWorld
    |-- CMakeLists.txt            # Link libraries
    |__
```
### Steps to launch the simulation

#### Step 1 Update and upgrade the Workspace image
```sh
$ sudo apt-get update
$ sudo apt-get upgrade -y
```

#### Step 2 Clone the  folder in /home/workspace/
```sh
$ cd /home/workspace/
$ git clone https://github.com/schandrachary/Robotics myrobot
$ git checkout -b robot_world origin/robot_world
```

#### Step 3 Compile the code
```sh
$ cd /home/workspace/myrobot/
$ mkdir build
$ cd build/
$ cmake ../
$ make
```

#### Step 4 Add the library path to the Gazebo plugin path  
```sh
$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/workspace/myrobot/build
```

#### Step 5 Run the Gazebo World file  
```sh
$ cd /home/workspace/myrobot/world/
$ gazebo chakraWorld
```

### Output
A welcome message is printed on the terminal, "Welcome to Chakra's world". You should also see a building model and two instances of robot model loaded inside the Gazebo World. 
![alt_text](pictures/default_gzclient_camera(1)-2020-11-27T13_55_50.281946.jpg)
