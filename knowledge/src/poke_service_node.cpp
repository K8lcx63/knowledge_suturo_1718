#include "ros/ros.h"
#include "object_detection/PokeObject.h"
#include "json_prolog/prolog.h"
#include <string>
#include <sstream>
  
using namespace json_prolog;

bool calculate_poke_position(object_detection::PokeObject::Request  &req, object_detection::PokeObject::Response &res)
{
    ROS_INFO("%g", req.detection.position.point.x);
    ROS_INFO("%g", req.detection.position.point.y);
    ROS_INFO("%g", req.detection.position.point.z);

	  std::stringstream ss;
	  ss << "calculate_poke_position(" << req.detection.position.point.x << "," 
	                                   << req.detection.position.point.y << "," 
	                                   << req.detection.position.point.z << ",RX,RY,RZ)";
	  std::string query = ss.str();

    Prolog pl;
  	PrologBindings bdg = pl.once(query);

    if(&bdg != NULL)
    {
        res.poke_position.header = req.detection.position.header;
      	res.poke_position.point.x = bdg["RX"];
      	res.poke_position.point.y = bdg["RY"];
    	res.poke_position.point.z = bdg["RZ"];

        return true;
    }
    else
    {
      return false;
    }
}
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "poke_service_node");
    ros::NodeHandle nh("~");
   
    ros::ServiceServer service = nh.advertiseService("calculate_poke_position", calculate_poke_position);
    ROS_INFO("Ready to calc.");
    ros::spin();
   
    return 0;
}