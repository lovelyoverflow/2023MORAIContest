# 2023MORAIContest

### Requirements
```
rosdep install --from-paths . --ignore-src -r -y
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