cmake_minimum_required(VERSION 2.8.3)
project(knowledge)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	poke_position_calculation
	kitchen_model_export
	json_prolog
	)


catkin_package(
CATKIN_DEPENDS 
	poke_position_calculation
	kitchen_model_export
	json_prolog
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)