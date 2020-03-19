# Robotic Software Engineer ND - Project 2: Go Chase It!

Simple ROS packages to make a small mobile robot chase a white ball.

## How To Use

### Clone repo as catkin_ws, initialize workspace and make
```
$ git clone https://github.com/civcode/RobotND-P2-GoChaseIt.git catkin_ws
$ cd catkin_ws/src 
$ catkin_init_workspace
$ cd .. && catkin_make
```

### Lauch ROS and Gazebo
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
