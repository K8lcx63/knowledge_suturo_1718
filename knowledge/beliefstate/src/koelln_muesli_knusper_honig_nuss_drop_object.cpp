#include <ros/ros.h>
#include <knowledge_msgs/DropObject.h>
#include <knowledge_msgs/Gripper.h>
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "koelln_muesli_knusper_honig_nuss_drop_object");
    ros::NodeHandle nh("~");

    ros::Publisher drop_pub = nh.advertise<knowledge_msgs::DropObject>("/beliefstate/drop_action", 1000);

    ros::Rate poll_rate(10);
    while(drop_pub.getNumSubscribers() < 1)
    {
        poll_rate.sleep();
        ROS_INFO_STREAM("wait...");
    }

    knowledge_msgs::DropObject drop_msg;
    drop_msg.gripper.gripper = knowledge_msgs::Gripper::RIGHT_GRIPPER;
    drop_pub.publish(drop_msg);

    return 0;
}