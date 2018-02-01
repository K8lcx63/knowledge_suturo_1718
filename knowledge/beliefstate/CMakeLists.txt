cmake_minimum_required(VERSION 2.8.3)
project(beliefstate)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	knowrob_common
	knowledge
	json_prolog
	knowledge_msgs
	geometry_msgs
	tfjava
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	knowledge
	json_prolog
	knowledge_msgs
	geometry_msgs
	tfjava
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(beliefstate_service src/beliefstate_service.cpp)
target_link_libraries(beliefstate_service ${catkin_LIBRARIES})

add_executable(perceive_object src/perceive_object.cpp)
target_link_libraries(perceive_object ${catkin_LIBRARIES})