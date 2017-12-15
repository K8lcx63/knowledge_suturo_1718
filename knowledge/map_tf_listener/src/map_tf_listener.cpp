#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_tf_listener");

	ros::NodeHandle node;
	tf::TransformListener listener;
  geometry_msgs::TransformStamped to_be_published;
  ros::Publisher pub = node.advertise<geometry_msgs::TransformStamped>("map_tf_listener_topic", 5);

	ros::Rate rate(10.0);
  	while (node.ok()){
    	tf::StampedTransform transform;
    	try{
      		listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
          tf::transformStampedTFToMsg(transform, to_be_published);
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