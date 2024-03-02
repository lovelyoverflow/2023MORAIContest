**2023 가상환경 기반 자율주행 경진대회**

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white) ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=for-the-badge&logo=ROS)

## 팀원

<table><tr><td align="center"><a href="https://github.com/lovelyoverflow"><img src="https://avatars.githubusercontent.com/u/14028864?v=4" width="100px;" alt=""/><br /><sub><b>박재훈</b></sub></a><br />SLAM, Navigation</td><td align="center"><a href="https://github.com/gyeongseoMin
"><img src="https://avatars.githubusercontent.com/u/67200721?v=4" width="100px;" alt=""/><br /><sub><b>민경서</b></sub></a><br />라이다, 장애물 회피</td>
    <td align="center"><a href="https://github.com/SeoooooNyeong"><img src="https://avatars.githubusercontent.com/u/113419106?v=4" width="100px;" alt=""/><br /><sub><b>안선영</b></sub></a><br />라이다, 장애물 회피</td><td align="center"><a href="https://github.com/JOONHOGITHUB"><img src="https://avatars.githubusercontent.com/u/105336903?v=4" width="100px;" alt=""/><br /><sub><b>이준호</b></sub></a><br />카메라, 차선 인식</td>
 <td align="center"><a href="https://github.com/leeharam2004"><img src="https://avatars.githubusercontent.com/u/44737337?v=4" width="100px;" alt=""/><br /><sub><b>이하람</b></sub></a><br />카메라, 차선 인식</td>
 
  </tr>
</table>
 
  </tr>
</table>

## SLAM & Navigation
![slam](https://github.com/lovelyoverflow/2023MORAIContest/assets/14028864/3a26fe4e-3dfc-4fd2-8293-4f4d36f1ae24)

## 장애물 회피
![obstacle](https://github.com/lovelyoverflow/2023MORAIContest/assets/14028864/a3368680-c9de-4b0e-9c85-cb0ecc9ce440)

## 차선 주행
![line_tracing](https://github.com/lovelyoverflow/2023MORAIContest/assets/14028864/55e1aaf7-95bd-4be4-b7ed-13f069161373)

## 교차로
![rotary](https://github.com/lovelyoverflow/2023MORAIContest/assets/14028864/dfc11ff2-5881-4d9b-81b9-5ab3ad059d8e)

## 신호등
![traffic_light](https://github.com/lovelyoverflow/2023MORAIContest/assets/14028864/1823bb35-dc9a-450a-97d0-51d4f838fceb)


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

### Run
```
$ source ./devel/setup.zsh
or
$ source ./devel/setup.bash

$ rosrun main main.launch
```


