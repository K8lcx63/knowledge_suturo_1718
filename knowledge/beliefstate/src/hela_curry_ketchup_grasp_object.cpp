#include <ros/ros.h>
#include <knowledge_msgs/GraspObject.h>
#include <knowledge_msgs/Gripper.h>
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hela_curry_ketchup_grasp_object");
    ros::NodeHandle nh("~");

    ros::Publisher grasp_pub = nh.advertise<knowledge_msgs::GraspObject>("/beliefstate/grasp_action", 1000);

    ros::Rate poll_rate(10);
    while(grasp_pub.getNumSubscribers() < 1)
    {
        poll_rate.sleep();
        ROS_INFO_STREAM("wait...");
    }

    knowledge_msgs::GraspObject grasp_msg;
    grasp_msg.object_label = "HelaCurryKetchup";
    grasp_msg.gripper.gripper = knowledge_msgs::Gripper::RIGHT_GRIPPER;
    grasp_pub.publish(grasp_msg);

    return 0;
}
