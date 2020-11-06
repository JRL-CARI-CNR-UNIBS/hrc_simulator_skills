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

add_executable(        node src/node.cpp)
add_dependencies(      node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( node
   ${catkin_LIBRARIES}
)

add_executable(        test_sincro src/test_sincro.cpp)
add_dependencies(      test_sincro ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( test_sincro
   ${catkin_LIBRARIES}
)


add_executable(        test_asincro src/test_asincro.cpp)
add_dependencies(      test_asincro ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( test_asincro
   ${catkin_LIBRARIES}
)
