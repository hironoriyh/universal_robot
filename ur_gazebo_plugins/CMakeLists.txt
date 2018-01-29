cmake_minimum_required(VERSION 2.8.3)
project(ur_gazebo_plugins)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES InitialPosePlugin
  CATKIN_DEPENDS
    gazebo_plugins
  DEPENDS
    gazebo
)

add_library(InitialPosePlugin src/InitialPosePlugin.cpp)
target_link_libraries(InitialPosePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(InitialPosePlugin handle_msgs_gencpp)

install(TARGETS
  InitialPosePlugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
