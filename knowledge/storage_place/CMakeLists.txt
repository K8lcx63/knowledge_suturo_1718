cmake_minimum_required(VERSION 2.8.3)
project(storage_place)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    knowrob_common
    knowledge_common
    json_prolog
    knowledge_msgs
    geometry_msgs
    tf
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
    knowrob_common
    knowledge_common
    json_prolog
    knowledge_msgs
    geometry_msgs
    tf
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(storage_place_service src/storage_place_service.cpp)
target_link_libraries(storage_place_service ${catkin_LIBRARIES})

add_executable(test_client src/test_client.cpp)
target_link_libraries(test_client ${catkin_LIBRARIES})