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
          ROS_INFO_STREAM("position: ");
          ROS_INFO_STREAM("\t x: " << srv.response.poses.at(i).position.x);
          ROS_INFO_STREAM("\t y: " << srv.response.poses.at(i).position.y);
          ROS_INFO_STREAM("\t z: " << srv.response.poses.at(i).position.z);
          ROS_INFO_STREAM("orientation: ");
          ROS_INFO_STREAM("\t x: " << srv.response.poses.at(i).orientation.x);
          ROS_INFO_STREAM("\t y: " << srv.response.poses.at(i).orientation.y);
          ROS_INFO_STREAM("\t z: " << srv.response.poses.at(i).orientation.z);
          ROS_INFO_STREAM("\t w: " << srv.response.poses.at(i).orientation.z);
          ROS_INFO_STREAM("bounding_box: ");
          ROS_INFO_STREAM("\t width: " << srv.response.bounding_boxes.at(i).x);
          ROS_INFO_STREAM("\t height: " << srv.response.bounding_boxes.at(i).y);
          ROS_INFO_STREAM("\t depth: " << srv.response.bounding_boxes.at(i).z);
          ROS_INFO_STREAM("mesh_path: " << srv.response.mesh_paths.at(i));
     }
     
     return 0;
}