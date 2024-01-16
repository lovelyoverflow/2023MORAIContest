# 2023MORAIContest

### Requirements
```
$ sudo apt install ros-noetic-rosbridge-server
$ sudo apt install ros-noetic-ackermann-msgs
$ sudo apt install ros-noetic-velodyne
$ sudo apt install ros-noetic-serial
```

### Installation
```
$ git clone --recurse-submodules git@github.com:lovelyoverflow/2023MORAIContest.git
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