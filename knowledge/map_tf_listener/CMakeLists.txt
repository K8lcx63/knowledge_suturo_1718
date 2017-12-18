cmake_minimum_required(VERSION 2.8.3)
project(map_tf_listener)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	tf
	geometry_msgs
	)

catkin_package(
CATKIN_DEPENDS 
	roscpp
	tf
	geometry_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(map_tf_listener src/map_tf_listener.cpp)
target_link_libraries(map_tf_listener ${catkin_LIBRARIES})
