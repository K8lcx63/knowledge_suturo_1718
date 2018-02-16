cmake_minimum_required(VERSION 2.8.3)
project(knowledge)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    marker_publisher
    interactiv_marker_publisher
    geometry_msgs
	json_prolog
	)


catkin_package(
CATKIN_DEPENDS 
    roscpp
    marker_publisher
    interactiv_marker_publisher
    geometry_msgs
	json_prolog
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(marker src/marker.cpp)
target_link_libraries(marker ${catkin_LIBRARIES})

add_executable(interactiv_marker src/interactiv_marker.cpp)
target_link_libraries(interactiv_marker ${catkin_LIBRARIES})