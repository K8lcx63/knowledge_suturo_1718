cmake_minimum_required(VERSION 2.8.3)
project(place_object)

find_package(catkin REQUIRED COMPONENTS 
	rospy
	knowrob_common
	knowledge_common
	json_prolog
	knowledge_msgs
	geometry_msgs
	beliefstate
	)


catkin_package(
CATKIN_DEPENDS 
	rospy
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