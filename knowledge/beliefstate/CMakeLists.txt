cmake_minimum_required(VERSION 2.8.3)
project(beliefstate)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	knowrob_common
	knowledge_common
	json_prolog
	knowledge_msgs
	geometry_msgs
	)


catkin_package(
CATKIN_DEPENDS 
	roscpp
	knowrob_common
	knowledge_common
	json_prolog
	knowledge_msgs
	geometry_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(ja_milch_perceive_object src/ja_milch_perceive_object.cpp)
target_link_libraries(ja_milch_perceive_object ${catkin_LIBRARIES})

add_executable(ja_milch_top_grasp_object src/ja_milch_top_grasp_object.cpp)
target_link_libraries(ja_milch_top_grasp_object ${catkin_LIBRARIES})

add_executable(ja_milch_side_grasp_object src/ja_milch_side_grasp_object.cpp)
target_link_libraries(ja_milch_side_grasp_object ${catkin_LIBRARIES})

add_executable(ja_milch_drop_object src/ja_milch_drop_object.cpp)
target_link_libraries(ja_milch_drop_object ${catkin_LIBRARIES})

add_executable(hela_curry_ketchup_perceive_object src/hela_curry_ketchup_perceive_object.cpp)
target_link_libraries(hela_curry_ketchup_perceive_object ${catkin_LIBRARIES})

add_executable(hela_curry_ketchup_grasp_object src/hela_curry_ketchup_grasp_object.cpp)
target_link_libraries(hela_curry_ketchup_grasp_object ${catkin_LIBRARIES})

add_executable(hela_curry_ketchup_drop_object src/hela_curry_ketchup_drop_object.cpp)
target_link_libraries(hela_curry_ketchup_drop_object ${catkin_LIBRARIES})

add_executable(kellogs_toppas_mini_perceive_object src/kellogs_toppas_mini_perceive_object.cpp)
target_link_libraries(kellogs_toppas_mini_perceive_object ${catkin_LIBRARIES})

add_executable(kellogs_toppas_mini_grasp_object src/kellogs_toppas_mini_grasp_object.cpp)
target_link_libraries(kellogs_toppas_mini_grasp_object ${catkin_LIBRARIES})

add_executable(kellogs_toppas_mini_drop_object src/kellogs_toppas_mini_drop_object.cpp)
target_link_libraries(kellogs_toppas_mini_drop_object ${catkin_LIBRARIES})