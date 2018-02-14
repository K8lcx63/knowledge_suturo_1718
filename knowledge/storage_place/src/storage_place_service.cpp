#include <string>
#include <sstream>
#include <ros/ros.h>
#include <knowledge_msgs/StoragePlace.h>
#include <knowledge_common/prolog_util.h>
#include <json_prolog/prolog.h>
  
using namespace json_prolog;

std::string createQuery(std::string object_label)
{
  std::stringstream ss;
    ss << "storage_place(suturo_object:\'" 
       << object_label << "\',"
       << "Position, Width, Height)";
    
  return ss.str();
}

bool find_storage_place(knowledge_msgs::StoragePlace::Request &req, knowledge_msgs::StoragePlace::Response &res)
{
    Prolog pl;
    PrologBindings bdg = pl.once(createQuery(req.object_label));

    if(&bdg != NULL)
    {
      res.storage_place_position = PrologUtil::prologBindingToPoint(bdg, "Position", "/map");
      res.storage_place_width = PrologUtil::prologValueToDouble(bdg["Width"]);
      res.storage_place_height = PrologUtil::prologValueToDouble(bdg["Height"]);

      return true;
    } 
    return false; 
}
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "storage_place");
  ros::NodeHandle nh("~");
   
  ros::ServiceServer storage_place_service = nh.advertiseService("storage_place", find_storage_place);

  ros::spin();
   
  return 0;
}