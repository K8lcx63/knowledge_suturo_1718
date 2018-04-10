#include <ros/ros.h>
#include <knowledge_msgs/GraspObject.h>
#include <knowledge_msgs/Gripper.h>
#include <geometry_msgs/PoseStamped.h>
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ja_milch_top_grasp_object");
    ros::NodeHandle nh("~");

    ros::Publisher grasp_pub = nh.advertise<knowledge_msgs::GraspObject>("/beliefstate/grasp_action", 1000);

    ros::Rate poll_rate(10);
    while(grasp_pub.getNumSubscribers() < 1)
    {
        poll_rate.sleep();
        ROS_INFO_STREAM("wait...");
    }

    knowledge_msgs::GraspObject grasp_msg;
    grasp_msg.object_label = "JaMilch";
    grasp_msg.gripper.gripper = knowledge_msgs::Gripper::LEFT_GRIPPER;
    grasp_msg.grasp_pose.header.frame_id = "ja_milch";
    grasp_msg.grasp_pose.header.stamp = ros::Time::now();
    grasp_msg.grasp_pose.pose.position.x = 0.009;
    grasp_msg.grasp_pose.pose.position.y = 0.001;
    grasp_msg.grasp_pose.pose.position.z = 0.070;
    grasp_msg.grasp_pose.pose.orientation.x = -0.0331417;
    grasp_msg.grasp_pose.pose.orientation.y = 0.687865;
    grasp_msg.grasp_pose.pose.orientation.z = 0.0245859;
    grasp_msg.grasp_pose.pose.orientation.w = 0.724664;
    grasp_pub.publish(grasp_msg);

    return 0;
}
