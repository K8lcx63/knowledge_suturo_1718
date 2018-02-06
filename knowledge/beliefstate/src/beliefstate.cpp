#include "string"
#include "sstream"
#include <ros/ros.h>
#include <knowledge_msgs/PerceivedObject.h>
//#include <knowledge_msgs/EmptyGripper.h>
#include <knowledge_msgs/Classify.h>
#include <knowledge_msgs/GraspObject.h>
#include <knowledge_msgs/DropObject.h>
#include <knowledge/prolog_util.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <json_prolog/prolog.h>
  
using namespace json_prolog;

ros::ServiceClient classify_service_client;

bool test_mode;
std::string dummy_class;

std::string gripper(const knowledge_msgs::Gripper gripper_msg)
{
  if(gripper_msg.gripper == knowledge_msgs::Gripper::LEFT_GRIPPER)
  {
    return "left_gripper";
  } 
  else
  {
    if(gripper_msg.gripper == knowledge_msgs::Gripper::RIGHT_GRIPPER)
    {
      return "right_gripper";
    }
    else
    {
      //this should not be possible
      return NULL;
    }
  }
}

std::string createQuery(geometry_msgs::PoseStamped object_pose, std::string object_label)
{
    std::stringstream ss;
    ss << "process_perceive_action(suturo_object:\'" 
       << PrologUtil::toCamelCase(object_label) << "\',"
       << PrologUtil::poseToPrologList(object_pose.pose) << ","
       << "\'" << object_pose.header.frame_id << "\')";
    
    return ss.str();
}

std::string createQuery(const knowledge_msgs::GraspObject grasp_object_msg)
{
    std::stringstream ss;
    ss << "process_grasp_action(suturo_object:" 
       << PrologUtil::toCamelCase(grasp_object_msg.object_label) << ","
       << "suturo_action: " << gripper(grasp_object_msg.gripper) << ")";
    return ss.str();
}

std::string createQuery(const knowledge_msgs::DropObject drop_object_msg)
{
    std::stringstream ss;
    ss << "process_drop_action(suturo_action:"
       << gripper(drop_object_msg.gripper) << ")";
    
    return ss.str();
}

std::string createQuery(int gripper)
{
  if(knowledge_msgs::Gripper::LEFT_GRIPPER == gripper)
  {
    return "object_attached_to_gripper(suturo_action:\'left_gripper\', ObjectIndividual)";
  }
  else
  {
    if(knowledge_msgs::Gripper::RIGHT_GRIPPER == gripper)
    {
      return "object_attached_to_gripper(suturo_action:\'right_gripper\', ObjectIndividual)";
    }
    else
    {
      //this should not be possible
      return NULL;
    }    
  }
}

void process_drop_action(const knowledge_msgs::DropObject &drop_object_msg)
{
  Prolog pl;
  PrologBindings bdg = pl.once(createQuery(drop_object_msg));

    ROS_INFO_STREAM("Dropped Object holding in " << gripper(drop_object_msg.gripper));
}

void process_grasp_action(const knowledge_msgs::GraspObject &grasp_object_msg)
{
  Prolog pl;
  PrologBindings bdg = pl.once(createQuery(grasp_object_msg));

  ROS_INFO_STREAM("Object " << grasp_object_msg.object_label << " grasped with " << gripper(grasp_object_msg.gripper));
}

bool process_perveive_action(knowledge_msgs::PerceivedObject::Request  &req, knowledge_msgs::PerceivedObject::Response &res)
{
  if(test_mode)
  {
    std::string object_label = dummy_class;

    Prolog pl;
    PrologBindings bdg = pl.once(createQuery(req.object_pose, object_label));

    ROS_INFO_STREAM("Perceived Object " << object_label);
    res.label = object_label;
    return &bdg != NULL;  
  }
  else
  {
    knowledge_msgs::Classify classify_srv;
    classify_srv.request.features = req.features;

    if(classify_service_client.call(classify_srv))
    {
      std::string object_label = classify_srv.response.label;

      Prolog pl;
      PrologBindings bdg = pl.once(createQuery(req.object_pose, object_label));

      ROS_INFO_STREAM("Perceived Object " << object_label);
      res.label = object_label;
      return &bdg != NULL;
    }
    return false;
  }
}

/*bool gripper_empty(knowledge_msgs::EmptyGripper::Request  &req, knowledge_msgs::EmptyGripper::Response &res)
{
  Prolog pl;
  PrologBindings bdg_left_gripper = pl.once(createQuery(knowledge_msgs::Gripper::LEFT_GRIPPER));
  PrologBindings bdg_right_gripper = pl.once(createQuery(knowledge_msgs::Gripper::RIGHT_GRIPPER));

  res.left_gripper = &bdg_left_gripper == NULL;
  res.right_gripper = &bdg_right_gripper == NULL;

  return true;
}*/
   
int main(int argc, char **argv)
{
  ros::init(argc, argv, "beliefstate");
  ros::NodeHandle nh("~");
   
  classify_service_client = nh.serviceClient<knowledge_msgs::Classify>("/svm_classifier/classify_service");
  ros::ServiceServer beliefstate_service = nh.advertiseService("perceive_action", process_perveive_action);
  ros::Subscriber graspActionSubscriber = nh.subscribe("grasp_action", 1000, process_grasp_action);
  ros::Subscriber dropActionSubscriber = nh.subscribe("drop_action", 1000, process_drop_action);
  //ros::ServiceServer gripper_service = nh.advertiseService("gripper_empty", gripper_empty);

  if (!nh.getParam("test_mode", test_mode))
  {
    ROS_ERROR("Could not find parameter 'test_mode' in namespace '%s'",
    nh.getNamespace().c_str());
    return 0;
  }

  if (!nh.getParam("dummy_class", dummy_class))
  {
    ROS_ERROR("Could not find parameter 'dummy_class' in namespace '%s'",
    nh.getNamespace().c_str());
    return 0;
  }

  ros::spin();
   
  return 0;
}