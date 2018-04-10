cmake_minimum_required(VERSION 2.8.3)
project(knowledge_grasp_verify)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	knowrob_common
	knowledge_common
	json_prolog
	knowledge_msgs
	geometry_msgs
	beliefstate
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	knowledge_common
	json_prolog
	knowledge_msgs
	geometry_msgs
	beliefstate
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(knowledge_grasp_verify src/knowledge_grasp_verification.cpp)
target_link_libraries(knowledge_grasp_verify ${catkin_LIBRARIES})