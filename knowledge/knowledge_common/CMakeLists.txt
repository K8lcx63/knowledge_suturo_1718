cmake_minimum_required(VERSION 2.8.3)
project(knowledge_common)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})