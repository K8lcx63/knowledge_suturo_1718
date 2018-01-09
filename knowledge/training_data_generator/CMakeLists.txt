cmake_minimum_required(VERSION 2.8.3)
project(training_data_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	pcl_conversions
	pcl_ros
	roscpp
	rospy
	sensor_msgs
	pr2_controllers_msgs
	actionlib
)

catkin_python_setup()

 add_service_files(
   FILES
	 GetNormals.srv
 )

 generate_messages(
   DEPENDENCIES
	sensor_msgs
 )


catkin_package(
CATKIN_DEPENDS 
	message_runtime 
	pcl_conversions
	pcl_ros
	roscpp
	rospy
	sensor_msgs
	pr2_controllers_msgs
	actionlib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(feature_extractor src/feature_extractor.cpp)
target_link_libraries(feature_extractor ${catkin_LIBRARIES})

add_executable(move_pr2_head src/move_pr2_head.cpp)
target_link_libraries(move_pr2_head ${catkin_LIBRARIES})