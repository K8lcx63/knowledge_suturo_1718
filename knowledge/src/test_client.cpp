#include "ros/ros.h"
#include "object_detection/PokeObject.h"
#include <string>
#include <iostream>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_client");
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<object_detection::PokeObject>("/poke_service_node/calculate_poke_position");

     object_detection::PokeObject srv;
     srv.request.detection.position.point.x = 7.368364604189992e-4;
     srv.request.detection.position.point.y = 0.17276005446910858e0;
     srv.request.detection.position.point.z = 0.9788062572479248e0;

     double a = (7.368364604189992e-4) -0.15;
     ROS_INFO("a: %g", a);

     if (client.call(srv))
     {
       ROS_INFO("x: %g", srv.response.poke_position.point.x);
       ROS_INFO("y: %g", srv.response.poke_position.point.y);
       ROS_INFO("z: %g", srv.response.poke_position.point.z);
     }
     else
     {
       ROS_ERROR("Failed to call service!");
       return 1;
     }
     return 0;
}