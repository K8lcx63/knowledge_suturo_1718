cmake_minimum_required(VERSION 2.8.3)
project(knowledge)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    marker_publisher
    geometry_msgs
	json_prolog
	)


catkin_package(
INCLUDE_DIRS include
LIBRARIES prolog_util
CATKIN_DEPENDS 
    roscpp
    marker_publisher
    geometry_msgs
	json_prolog
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(prolog_util src/prolog_util)

add_executable(marker src/marker.cpp)
target_link_libraries(marker ${catkin_LIBRARIES})