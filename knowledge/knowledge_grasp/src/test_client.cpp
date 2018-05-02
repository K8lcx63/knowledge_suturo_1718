#include "ros/ros.h"
#include "knowledge_msgs/GraspIndividual.h"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_client");
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<knowledge_msgs::GraspIndividual>("/knowledge_grasp/knowledge_grasp");

     knowledge_msgs::GraspIndividual srv;
     srv.request.object_label = "JaMilch";

     client.call(srv);
     //ROS_INFO_STREAM("x: " << srv.response.grasp_pose.pose.position.x); 
     //ROS_INFO_STREAM("y: " << srv.response.grasp_pose.pose.position.y); 
     //ROS_INFO_STREAM("z: " << srv.response.grasp_pose.pose.position.z); 
     //ROS_INFO_STREAM("x: " << srv.response.grasp_pose.pose.orientation.x); 
     //ROS_INFO_STREAM("y: " << srv.response.grasp_pose.pose.orientation.y); 
     //ROS_INFO_STREAM("z: " << srv.response.grasp_pose.pose.orientation.z); 
     //ROS_INFO_STREAM("w: " << srv.response.grasp_pose.pose.orientation.w);
     
     return 0;
}