#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_tf_listener");

	ros::NodeHandle node;
	tf::TransformListener listener;
  ros::Publisher pub = node.advertise<geometry_msgs::PointStamped>("map_tf_listener_topic", 5);

	ros::Rate rate(10.0);
  	while (node.ok()){
    	geometry_msgs::PointStamped to_be_published;
      geometry_msgs::PointStamped point;
      point.header.stamp = ros::Time(0);
      point.header.frame_id = "/base_link";
      point.point.x = point.point.y = point.point.z = 0;
    	try{
      		listener.transformPoint("/map", point, to_be_published);
          pub.publish(to_be_published);
    	}
    	catch (tf::TransformException &ex) {
     	 	ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
      	continue;
    	}
    	
    	rate.sleep();
    }
	return 0;
}