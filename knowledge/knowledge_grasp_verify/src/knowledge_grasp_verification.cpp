#include <string>
#include <sstream>
#include <locale>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <knowledge_msgs/GraspIndividual.h>
#include <knowledge_common/prolog_util.h>
#include <json_prolog/prolog.h>
  
using namespace json_prolog;

geometry_msgs::PoseArray arrayToPub;

std::string createQuery(std::string object_label)
{
  std::stringstream ss;
    ss << "find_grasp_pose(suturo_object:\'" 
       << object_label << "\',"
       << "Position, Quaternion)";
    
  return ss.str();
}

std::string translateToUnderscore(std::string label){

  std::stringstream rs;

  rs << std::tolower(label.at(0), std::locale());

  for(unsigned i=1; i<label.length(); ++i){
    if(std::isupper(label.at(i))){
      rs << "_" << std::tolower(label.at(i), std::locale());
    } else {
      rs << label.at(i);
    }
  }
  return rs.str();

}

bool find_grasp_poses(std::string object_label)
{
    Prolog pl;
    PrologQueryProxy bdgs = pl.query(createQuery(object_label));
 
 	arrayToPub.poses.clear();
  for(PrologQueryProxy::iterator it=bdgs.begin();
      it != bdgs.end(); it++)
    {
      PrologBindings bdg = *it;
	  geometry_msgs::Pose poseToAdd = PrologUtil::prologBindingToPose(bdg, "Position", "Quaternion");

	  arrayToPub.poses.push_back(poseToAdd);
	  }
    return true;
}
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "knowledge_grasp_verify");
  ros::NodeHandle nh;
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseArray>("poses", 1000);
  ros::Duration(30.0).sleep();
  
  	  arrayToPub.header.frame_id = "/ja_milch";
  find_grasp_poses("SiggBottle");
  int count = 0;
  ros::Rate loop_rate(10);
  while(ros::ok()){
  	  
  	  

  	  arrayToPub.header.stamp = ros::Time::now();

  	  pose_pub.publish(arrayToPub);
  	  ros::spinOnce();
  	  loop_rate.sleep();
  	  ++count;
  }
   
  return 0;
}
