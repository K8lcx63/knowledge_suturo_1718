#include "string"
#include "sstream"
#include "vector"
#include <ros/ros.h>
#include <knowledge_msgs/ProcessPercivedObject.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <json_prolog/prolog.h>
#include <tf/tf.h>
  
using namespace json_prolog;

const std::string OBJECT_ID = "ObjectId";

std::string quaternionToPrologList(geometry_msgs::Quaternion quaternion)
{
	std::stringstream ss;
	ss << "[" << quaternion.x << ","
	          << quaternion.y << "," 
	          << quaternion.z << "," 
	          << quaternion.w << "]";

	return ss.str();
}

std::string pointToPrologList(geometry_msgs::Point point)
{
	std::stringstream ss;
	ss << "[" << point.x << ","
	          << point.y << "," 
	          << point.z << "]";
	return ss.str();
}

std::string createQuery(knowledge_msgs::ProcessPercivedObject::Request  &req)
{
    std::stringstream ss;
    ss << "belief_perceived_at(" 
       << req.object_type << "," 
       << "[ " << req.pose.header.frame_id << ", \'map\'],"
       << pointToPrologList(req.pose.pose.position) << ","
       << quaternionToPrologList(req.pose.pose.orientation) << ","
       << "[0.1,0.1],"
       <<  OBJECT_ID << "])";
    
    return ss.str();
}

bool process_percived_object(knowledge_msgs::ProcessPercivedObject::Request  &req, knowledge_msgs::ProcessPercivedObject::Response &res)
{
    std::string query = "belief_perceived_at(suturo_object:'PfannerIceTea2LPackage', ['map',_,[1.001,0.001,0.0],[1.0,0.0,0.0,0.0]], [0.0,0.0], Cup)";
    Prolog pl;
  	PrologBindings bdg = pl.once(query);

    if(&bdg)
    {
      res.object_id = bdg["Cup"].toString();
      return true;
    }
    else
    {
      return false;
    }

}
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "beliefstate_service");
    ros::NodeHandle nh("~");
   
    ros::ServiceServer service = nh.advertiseService("process_percived_object", process_percived_object);
    ros::spin();
   
    return 4;
}