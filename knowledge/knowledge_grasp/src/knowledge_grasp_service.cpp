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
       << "Position, Quaternion)";
    
  return ss.str();
}

bool find_grasp_pose(knowledge_msgs::GraspIndividual::Request &req, knowledge_msgs::GraspIndividual::Response &res)
{
    Prolog pl;
    PrologBindings bdg = pl.once(createQuery(req.object_label));

    if(&bdg != NULL)
    {
      if(req.object_label == "JaMilch"){res.grasp_pose.header.frame_id = "/ja_milch";}
      if(req.object_label == "HelaCurryKetchup"){res.grasp_pose.header.frame_id = "/hela_curry_ketchup";}
      if(req.object_label == "SiggBottle"){res.grasp_pose.header.frame_id = "/sigg_bottle";}
      if(req.object_label == "KeloggsToppasMini"){res.grasp_pose.header.frame_id = "/keloggs_toppas_mini";}
      if(req.object_label == "CupEcoOrange"){res.grasp_pose.header.frame_id = "/cup_eco_orange";}
      if(req.object_label == "EdekaRedBowl"){res.grasp_pose.header.frame_id = "/edeka_red_bowl";}
      if(req.object_label == "PringlesSalt"){res.grasp_pose.header.frame_id = "/pringles_salt";}
      if(req.object_label == "PringlesPaprika"){res.grasp_pose.header.frame_id = "/pringles_paprika";}
      if(req.object_label == "KoellnMuesliKnusperHonigNuss"){res.grasp_pose.header.frame_id = "/koelln_muesli_knusper_honig_nuss";}
      if(req.object_label == "TomatoSauceOroDiParma"){res.grasp_pose.header.frame_id = "/tomato_sauce_oro_di_parma";}
      //res.grasp_pose.header.frame_id = "/" + req.object_label;
	    res.grasp_pose.pose =  PrologUtil::prologBindingToPose(bdg, "Position", "Quaternion");
      return true;
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