cmake_minimum_required(VERSION 2.8.3)
project(knowledge_common)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    knowrob_common
    geometry_msgs
    json_prolog
	)


catkin_package(
INCLUDE_DIRS include
LIBRARIES prolog_util
CATKIN_DEPENDS 
    roscpp
    knowrob_common
    geometry_msgs
    json_prolog
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_library(prolog_util src/prolog_util)