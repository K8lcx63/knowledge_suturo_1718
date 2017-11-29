cmake_minimum_required(VERSION 2.8.3)
project(semantic_logging)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	knowrob_common
	json_prolog
	knowledge_msgs
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	json_prolog
	knowledge_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(logger_service src/logger_service.cpp)
target_link_libraries(logger_service ${catkin_LIBRARIES})

add_executable(logger_test_client src/logger_test_client.cpp)
target_link_libraries(logger_test_client ${catkin_LIBRARIES})