#include "ros/ros.h"
#include "object_detection/PokeObject.h"
#include <string>
#include <marker_publisher/marker_publisher.h>


int main(int argc, char **argv)
{
     ros::init(argc, argv, "test_client");
   
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<object_detection::PokeObject>("/poke_position_service/calculate_poke_position");

     geometry_msgs::PointStamped vision_point;
     vision_point.header.stamp = ros::Time();
     vision_point.header.frame_id = "/head_mount_kinect_ir_optical_frame";
     vision_point.point.x = 0.475767;;
     vision_point.point.y = -0.021934;
     vision_point.point.z = 1.453995;

     object_detection::PokeObject srv;
     srv.request.direction = object_detection::PokeObject::Request::DIRECTION_LEFT;
     srv.request.detection.position = vision_point; 

     client.call(srv);

     return 0;
}