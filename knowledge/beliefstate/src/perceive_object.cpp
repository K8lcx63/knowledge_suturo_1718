#include "string"
#include "sstream"
#include <ros/ros.h>
#include <knowledge_msgs/PerceivedObject.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "perceive_object");
  ros::NodeHandle nh("~");
   
  ros::ServiceClient  perceive_object_service = nh.serviceClient<knowledge_msgs::PerceivedObject>("/beliefstate/perceive_action");
  
  knowledge_msgs::PerceivedObject perceive_object_srv;
  geometry_msgs::PoseStamped object_pose;
  object_pose.header.stamp = ros::Time::now();
  object_pose.header.frame_id = "/head_mount_kinect_ir_optical_frame";
  object_pose.pose.position.x = 1.0;
  object_pose.pose.position.y = -2.5;
  object_pose.pose.position.z = 1.4;
  object_pose.pose.orientation.x = 0.0;
  object_pose.pose.orientation.y = 0.0;
  object_pose.pose.orientation.z = 0.0;
  object_pose.pose.orientation.w = 1.0;
  perceive_object_srv.request.object_pose = object_pose;

  if(perceive_object_service.call(perceive_object_srv))
  {
    ROS_INFO_STREAM("Perceived test succeded " << perceive_object_srv.response.label);
  }
  else
  {
    ROS_INFO_STREAM("Perceive test failed!!!!");
  }

  return 0;
}