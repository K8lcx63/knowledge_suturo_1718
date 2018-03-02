#include <string>
#include <sstream>
#include <ros/ros.h>
#include <knowledge_msgs/GraspIndividual.h>
#include <knowledge_common/prolog_util.h>
#include <json_prolog/prolog.h>
  
using namespace json_prolog;

std::string createQuery(std::string object_label)
{
  std::stringstream ss;
    ss << "find_grasp_pose(suturo_object:\'" 
       << object_label << "\',"
       << "Pose)";
    
  return ss.str();
}

bool find_grasp_pose(knowledge_msgs::GraspIndividual::Request &req, knowledge_msgs::GraspIndividual::Response &res)
{
    Prolog pl;
    PrologBindings bdg = pl.once(createQuery(req.object_label));

    if(&bdg != NULL)
    {
       res.grasp_pose.header.frame_id = "/" + req.object_label;
      
	     std::string s = bdg["Pose"];
        std::string::size_type sz;

        res.grasp_pose.pose.position.x = std::stod (s,&sz);
        res.grasp_pose.pose.position.y = std::stod (s.substr(sz),&sz);
        res.grasp_pose.pose.position.z = std::stod (s.substr(sz),&sz);

        res.grasp_pose.pose.orientation.x = std::stod (s.substr(sz),&sz);
        res.grasp_pose.pose.orientation.y = std::stod (s.substr(sz),&sz);
        res.grasp_pose.pose.orientation.z = std::stod (s.substr(sz),&sz);
        res.grasp_pose.pose.orientation.w = std::stod (s.substr(sz),&sz);
	  } 
    return false; 
}
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "knowledge_grasp");
  ros::NodeHandle nh("~");
   
  ros::ServiceServer knowledge_grasp_service = nh.advertiseService("knowledge_grasp", find_grasp_pose);

  ros::spin();
   
  return 0;
}