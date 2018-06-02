cmake_minimum_required(VERSION 2.8.3)
project(python_client)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs  urdf)

catkin_package(
    CATKIN_DEPENDS  urdf geometry_msgs sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

