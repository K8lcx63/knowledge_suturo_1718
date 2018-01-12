cmake_minimum_required(VERSION 2.8.3)
project(train_svm_classifier)

find_package(catkin REQUIRED COMPONENTS
	pcl_conversions
	pcl_ros
	rospy
)

catkin_python_setup()


catkin_package(
CATKIN_DEPENDS 
	pcl_conversions
	pcl_ros
	rospy
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)