cmake_minimum_required(VERSION 2.8.3)
project(hrc_mosaic_test)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  manipulation_msgs
  object_loader
  roscpp
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS manipulation_msgs object_loader roscpp
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

