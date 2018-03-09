#include <string>
#include <sstream>
#include <ros/ros.h>
#include <knowledge_msgs/StoragePlace.h>
#include <knowledge_common/prolog_util.h>
#include <json_prolog/prolog.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
  
using namespace json_prolog;

class StoragePlace{
private:
    tf::TransformListener listener;

    std::string createQuery(std::string object_label)
    {
        std::stringstream ss;
        ss << "storage_place(suturo_object:\'" 
           << object_label << "\',"
           << "Position, Width, Height)";
    
        return ss.str();
    }

public:
    bool find_storage_place(knowledge_msgs::StoragePlace::Request &req, knowledge_msgs::StoragePlace::Response &res)
    {
        Prolog pl;
        PrologBindings bdg = pl.once(createQuery(req.object_label));

        if(&bdg != NULL)
        {
            geometry_msgs::PointStamped sink_area_surface_point =  PrologUtil::prologBindingToPoint(bdg, "Position", "iai_kitchen/sink_area_surface");
            geometry_msgs::PointStamped map_point;
            try
            {
                sink_area_surface_point.header.stamp = ros::Time(0);
                listener.transformPoint("/map", sink_area_surface_point, map_point);
            }
            catch (tf::TransformException &ex) 
            {
                return false;
            }
            res.storage_place_position = map_point;
            res.storage_place_width = PrologUtil::prologValueToDouble(bdg["Width"]);
            res.storage_place_height = PrologUtil::prologValueToDouble(bdg["Height"]);

            return true;
        } 
        return false; 
    }
};
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "storage_place");
  ros::NodeHandle nh("~");

  StoragePlace storagePlace;
  ros::ServiceServer storage_place_service = nh.advertiseService("storage_place", &StoragePlace::find_storage_place, &storagePlace);

  ros::spin();
   
  return 0;
}