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
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	knowledge
	json_prolog
	knowledge_msgs
	geometry_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(beliefstate src/beliefstate.cpp)
target_link_libraries(beliefstate ${catkin_LIBRARIES})

add_executable(perceive_object src/perceive_object.cpp)
target_link_libraries(perceive_object ${catkin_LIBRARIES})

add_executable(grasp_object src/grasp_object.cpp)
target_link_libraries(grasp_object ${catkin_LIBRARIES})

#add_executable(empty_gripper src/empty_gripper.cpp)
#target_link_libraries(empty_gripper ${catkin_LIBRARIES})