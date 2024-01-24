# 2023MORAIContest

### Requirements
```
$ sudo apt install ros-noetic-rosbridge-server
$ sudo apt install ros-noetic-ackermann-msgs
$ sudo apt install ros-noetic-velodyne
$ sudo apt install ros-noetic-serial
$ sudo apt install ros-noetic-geographic-msgs
$ sudo apt install ros-noetic-map-server

$ rosdep install robot_localization
```

### Installation
```
$ git clone git@github.com:lovelyoverflow/2023MORAIContest.git
$ cd ./2023MORAIContest
$ catkin_make
```

### Test
```
$ source ./devel/setup.zsh
or
$ source ./devel/setup.bash

$ rosrun main image_parser.py
```

### 빌드 오류 시
devel 폴더를 지우고 재빌드