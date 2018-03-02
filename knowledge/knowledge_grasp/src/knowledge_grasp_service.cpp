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
       << "[[X1,Y1,Z1],[X2,Y2,Z2,W]])";
    
  return ss.str();
}

bool find_grasp_pose(knowledge_msgs::GraspIndividual::Request &req, knowledge_msgs::GraspIndividual::Response &res)
{
    Prolog pl;
    PrologBindings bdg = pl.once(createQuery(req.object_label));

    if(&bdg != NULL)
    {
       res.grasp_pose.header.frame_id = "/" + req.object_label;
      
	     res.grasp_pose.pose.position.x = bdg["X1"];
	     res.grasp_pose.pose.position.y = bdg["Y1"];
	     res.grasp_pose.pose.position.z = bdg["Z1"];
	  
	     res.grasp_pose.pose.orientation.x = bdg["X2"];
	     res.grasp_pose.pose.orientation.y = bdg["Y2"];
	     res.grasp_pose.pose.orientation.z = bdg["Z2"];
	     res.grasp_pose.pose.orientation.w = bdg["W"];
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