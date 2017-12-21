cmake_minimum_required(VERSION 2.8.3)
project(poke_position_calculation)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	knowrob_common
	json_prolog
	object_detection
	marker_publisher
	tf
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	json_prolog
	object_detection
	geometry_msgs
	marker_publisher
	tf
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(poke_position_service src/poke_position_service.cpp)
target_link_libraries(poke_position_service ${catkin_LIBRARIES})

add_executable(test_client_left src/test_client_left.cpp)
target_link_libraries(test_client_left ${catkin_LIBRARIES})

add_executable(test_client_right src/test_client_right.cpp)
target_link_libraries(test_client_right ${catkin_LIBRARIES})

add_executable(test_marker src/test_marker.cpp)
target_link_libraries(test_marker ${catkin_LIBRARIES})