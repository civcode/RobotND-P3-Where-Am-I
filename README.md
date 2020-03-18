# Robotic Software Engineer ND - Project 2: Go Chase It!

Simple ROS packages to make a small mobile robot chase a white ball.

## How To Use

### Clone repo to catkin_ws, initialize workspace and make
```
$ cd catkin_ws 
$ mkdir src && cd src
$ catkin_init_workspace
$ cd .. && catkin_make
```
### Run ROS and Gazebo
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
