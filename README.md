# 2023MORAIContest

### Requirements
```
$ sudo apt install ros-noetic-rosbridge-server
$ sudo apt install ros-noetic-velodyne
$ sudo apt install ros-noetic-serial
```

### Installation
```
$ git clone git@github.com:lovelyoverflow/2023MORAIContest.git
$ cd 2023MORAIContest/src
$ git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git
$ git clone https://github.com/sbgisen/vesc.git

$ cd ..
$ catkin_make
```

### Test
```
$ source ./devel/setup.zsh
or
$ source ./devel/setup.bash

$ rosrun main image_parser.py
```