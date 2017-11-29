cmake_minimum_required(VERSION 2.8.3)
project(poke_position_calculation)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	knowrob_common
	json_prolog
	object_detection
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	json_prolog
	object_detection
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(poke_position_service src/poke_position_service.cpp)
target_link_libraries(poke_position_service ${catkin_LIBRARIES})

add_executable(test_client src/test_client.cpp)
target_link_libraries(test_client ${catkin_LIBRARIES})