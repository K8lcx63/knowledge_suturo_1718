#include "ros/ros.h"
#include "knowledge_msgs/LogStartAction.h"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "logger_test_client");
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<knowledge_msgs::LogStartAction>("/logger_service/logStartAction");

     knowledge_msgs::LogStartAction srv;
     srv.request.action_type = "percive_object";
     srv.request.task_context = "try to percive the ice tea";
     srv.request.start_time = ros::Time::now();
     //no prev_action

     client.call(srv);


     
     return 0;
}