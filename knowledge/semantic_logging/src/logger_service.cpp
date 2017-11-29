#include "ros/ros.h"
#include "knowledge_msgs/LogStartAction.h"
#include "json_prolog/prolog.h"
#include <string>
#include <sstream>
  
using namespace json_prolog;

bool logStartAction(knowledge_msgs::LogStartAction::Request  &req, knowledge_msgs::LogStartAction::Response &res)
{
	ROS_INFO("log_start_action request:");
	ROS_INFO_STREAM("action_type: " << req.action_type);
	ROS_INFO_STREAM("task_context: " << req.task_context);
	ROS_INFO_STREAM("start_time: " << req.start_time);
	ROS_INFO_STREAM("prev_action: " << req.prev_action);

	std::stringstream ss;
	ss << "cram_start_action(" << req.action_type << "," 
							   << req.task_context << "," 
							   << req.start_time << ",";

	if(req.prev_action.empty())
	{
		ss << "PrevAction";
	}
	else
	{
		ss << req.prev_action;
	}

	ss << ", ActionInst)";
	std::string query = ss.str();

    Prolog pl;
  	PrologBindings bdg = pl.once(query);

    if(&bdg != NULL)
    {
    	ROS_INFO("log_start_action response");
    	ROS_INFO("success: true");
    	res.success = true;
    	return true;
    }
    else
    {
    	ROS_INFO("log_start_action response");
    	ROS_INFO("success: false");
    	res.success = false;
    	return false;
    }
}
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "logger_service");
    ros::NodeHandle nh("~");
   
    ros::ServiceServer service = nh.advertiseService("logStartAction", logStartAction);
    ros::spin();
   
    return 0;
}