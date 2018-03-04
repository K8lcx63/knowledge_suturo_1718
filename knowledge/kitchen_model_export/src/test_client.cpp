#include "ros/ros.h"
#include "knowledge_msgs/GetFixedKitchenObjects.h"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_client");
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<knowledge_msgs::GetFixedKitchenObjects>("/kitchen_model_service/get_fixed_kitchen_objects");

     knowledge_msgs::GetFixedKitchenObjects srv;

     client.call(srv);
     for(int i = 0; i < srv.response.names.size(); i++)
     {
          ROS_INFO_STREAM("name: " << srv.response.names.at(i));
          ROS_INFO_STREAM("path: " << srv.response.meshes.at(i));
          ROS_INFO_STREAM("frame: " << srv.response.frames.at(i));
     }
     
     return 0;
}