cmake_minimum_required(VERSION 2.8.3)
project(navigation_train)

find_package(catkin REQUIRED COMPONENTS
  navigation_gym
  rospy
)


catkin_package(

)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
