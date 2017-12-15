#include "ros/ros.h"
#include "knowledge_msgs/ProcessPercivedObject.h"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");
   
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<knowledge_msgs::ProcessPercivedObject>("/beliefstate_service/process_percived_object");

    knowledge_msgs::ProcessPercivedObject srv;
    srv.request.object_type = "knowrob:\'Cup\'";
    srv.request.pose.header.stamp = ros::Time::now();
    srv.request.pose.header.frame_id = "\'kinect_optical_frame\'";
    srv.request.pose.pose.position.x = 1.0;
    srv.request.pose.pose.position.y = 2.0;
    srv.request.pose.pose.position.z = 3.0;
    srv.request.pose.pose.orientation.x = 0.0;
    srv.request.pose.pose.orientation.y = 0.0;
    srv.request.pose.pose.orientation.z = 0.0;
    srv.request.pose.pose.orientation.w = 1.0;
    
    client.call(srv);

    ROS_INFO_STREAM("object_id: " << srv.response.object_id);
     
    return 0;
}