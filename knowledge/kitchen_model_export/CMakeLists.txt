cmake_minimum_required(VERSION 2.8.3)
project(kitchen_model_export)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	knowrob_common
	json_prolog
	knowledge_msgs
	geometry_msgs
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	json_prolog
	knowledge_msgs
	geometry_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(kitchen_model_service src/kitchen_model_service.cpp)
target_link_libraries(kitchen_model_service ${catkin_LIBRARIES})

add_executable(test_client src/test_client.cpp)
target_link_libraries(test_client ${catkin_LIBRARIES})