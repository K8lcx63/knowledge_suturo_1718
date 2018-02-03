#include <ros/ros.h>
#include <knowledge_msgs/EmptyGripper.h>
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "empty_gripper");
  ros::NodeHandle nh("~");
   
  ros::ServiceClient  empty_gripper_service = nh.serviceClient<knowledge_msgs::EmptyGripper>("/beliefstate/gripper_empty");
  
  knowledge_msgs::EmptyGripper empty_gripper_srv;

  if(empty_gripper_service.call(empty_gripper_srv))
  {
    ROS_INFO_STREAM("left gripper empty: " << empty_gripper_srv.response.left_gripper);
    ROS_INFO_STREAM("right gripper empty: " << empty_gripper_srv.response.right_gripper);
  }
  else
  {
    ROS_INFO_STREAM("gripper test failed!!!!");
  }

  return 0;
}