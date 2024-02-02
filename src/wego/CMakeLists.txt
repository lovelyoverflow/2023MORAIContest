cmake_minimum_required(VERSION 3.0.2)
project(wego)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wego
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_convert src/convert_lidar.cpp)
target_link_libraries(lidar_convert ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/pub_odom.py
  scripts/navigation_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)