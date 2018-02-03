#include <ros/ros.h>
#include <knowledge_msgs/GraspObject.h>
#include <knowledge_msgs/Gripper.h>
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_object");
  ros::NodeHandle nh("~");

  ros::Publisher grasp_pub = nh.advertise<knowledge_msgs::GraspObject>("/beliefstate/grasp_action", 1000);

  knowledge_msgs::GraspObject grasp_msg;
  grasp_msg.object_label = "hela_curry_ketchup";
  grasp_msg.gripper.gripper = knowledge_msgs::Gripper::LEFT_GRIPPER;
  grasp_pub.publish(grasp_msg);

  return 0;
}
