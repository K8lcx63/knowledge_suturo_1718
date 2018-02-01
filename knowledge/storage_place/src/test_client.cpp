#include "string"
#include "sstream"
#include <ros/ros.h>
#include <knowledge_msgs/StoragePlace.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_client");
  ros::NodeHandle nh("~");
   
  ros::ServiceClient storage_place_service = nh.serviceClient<knowledge_msgs::StoragePlace>("/storage_place_service/storage_place");
  
  const std::string object_label_array[] = {"hela_curry_ketchup", "tomato_sauce_oro_di_parma", 
                                            "pringles_paprika", "pringles_salt", "ja_milch", 
                                            "koelln_muesli_knusper_honig_nuss", "kelloggs_toppas_mini",
                                            "cup_eco_orange", "edeka_red_bowl", "sigg_bottle"};

  for(int i = 0; i < 10; i++)
  {
    knowledge_msgs::StoragePlace storage_place_srv;
    storage_place_srv.request.object_label = object_label_array[i];

    if(storage_place_service.call(storage_place_srv))
    {
      ROS_INFO_STREAM("object_label: " << object_label_array[i]);
      ROS_INFO_STREAM("x: " << storage_place_srv.response.storage_place_position.point.x);
      ROS_INFO_STREAM("y: " << storage_place_srv.response.storage_place_position.point.y);
      ROS_INFO_STREAM("z: " << storage_place_srv.response.storage_place_position.point.z);
      ROS_INFO_STREAM("w: " << storage_place_srv.response.storage_place_width);
      ROS_INFO_STREAM("h: " << storage_place_srv.response.storage_place_height);
    }
    else
    {
      ROS_INFO_STREAM(object_label_array[i] << " test failed!!!!");
    }
  }

  return 0;
}