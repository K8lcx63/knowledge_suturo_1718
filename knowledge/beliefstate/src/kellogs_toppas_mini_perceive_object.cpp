#include "string"
#include "sstream"
#include <ros/ros.h>
#include <knowledge_msgs/PerceivedObject.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kellogs_toppas_mini_perceive_object");
  ros::NodeHandle nh("~");
   
  ros::Publisher perceive_pub = nh.advertise<knowledge_msgs::PerceivedObject>("/beliefstate/perceive_action", 1000);
  
  ros::Rate poll_rate(10);
  while(perceive_pub.getNumSubscribers() < 1)
  {
    poll_rate.sleep();
    ROS_INFO_STREAM("wait...");
  }

  knowledge_msgs::PerceivedObject perceive_msg;
  geometry_msgs::PoseStamped object_pose;
  object_pose.header.stamp = ros::Time::now();
  object_pose.header.frame_id = "/map";
  object_pose.pose.position.x = 1.513453;
  object_pose.pose.position.y = 0.656326;
  object_pose.pose.position.z = 0.968618;
  object_pose.pose.orientation.x = 0.0;
  object_pose.pose.orientation.y = 0.0;
  object_pose.pose.orientation.z = 0.0;
  object_pose.pose.orientation.w = 1.0;

  perceive_msg.object_label = "KellogsToppasMini";
  perceive_msg.object_pose = object_pose;

  perceive_pub.publish(perceive_msg);

  return 0;
}