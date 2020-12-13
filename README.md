# Path Planning using A-Start - Visualization
You will visualize the shortest path for the robot to travel through the generated image

### Compiling
```sh
$ cd /home/workspace/
$ git clone https://github.com/udacity/RoboND-A-Visualization
$ cd /RoboND-A-Visualization/
$ rm -rf Images/* #Delete the folder content and not the folder itself!
$ g++ main.cpp -o app -std=c++11 -I/usr/include/python2.7 -lpython2.7
```

### Running
```sh
$ ./app
```

Now, wait for the program to generate the path!

### Generated Path

![alt text](Images/Path.png)

