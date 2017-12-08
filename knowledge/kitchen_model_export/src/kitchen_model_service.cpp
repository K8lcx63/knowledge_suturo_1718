#include "string"
#include "sstream"
#include "vector"
#include <ros/ros.h>
#include <knowledge_msgs/GetFixedKitchenObjects.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <json_prolog/prolog.h>
#include <tf/tf.h>
  
using namespace json_prolog;

std::string createQuery()
{
    std::stringstream ss;
    ss << "getFixedKitchenObjects(Object,";
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            ss << "M" << i << j << ",";
        }
    } 

    ss << "Width, Height, Depth)";
    ROS_INFO_STREAM("query: " << ss.str());
    return ss.str();
}

double prologValueToDouble(PrologBindings bdg, std::string identifier)
{
    double value;

    if(bdg[identifier].type() == PrologValue::value_type::DOUBLE)
    {
        value = bdg[identifier];
    }
    
    if(bdg[identifier].type() == PrologValue::value_type::INT)
    {
        int64_t temp = bdg[identifier];
        value = static_cast<double>(temp);
    }
    return value;
}

tf::Quaternion prologBindingToQuaternion(PrologBindings bdg)
{
    int64_t m00 = prologValueToDouble(bdg, "M00");
    int64_t m01 = prologValueToDouble(bdg, "M01");
    int64_t m02 = prologValueToDouble(bdg, "M02");
    int64_t m10 = prologValueToDouble(bdg, "M10");
    int64_t m11 = prologValueToDouble(bdg, "M11");
    int64_t m12 = prologValueToDouble(bdg, "M12");
    int64_t m20 = prologValueToDouble(bdg, "M20");
    int64_t m21 = prologValueToDouble(bdg, "M21");
    int64_t m22 = prologValueToDouble(bdg, "M22");

    tf::Matrix3x3 rotationMatrix(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    tf::Quaternion quaternion;
    rotationMatrix.getRotation(quaternion);

    return quaternion;
}

tf::Vector3 prologBindingToPosition(PrologBindings bdg)
{
    double m03 = prologValueToDouble(bdg, "M03");
    double m13 = prologValueToDouble(bdg, "M13");
    double m23 = prologValueToDouble(bdg, "M23");
    
    return tf::Vector3(m03, m13, m23);
}

geometry_msgs::Vector3 toBoundingBoxMsgs(PrologBindings bdg)
{
    double width = prologValueToDouble(bdg, "Width");
    double height = prologValueToDouble(bdg, "Height");
    double depth = prologValueToDouble(bdg, "Depth");

    geometry_msgs::Vector3 boundingBoxMsgs;
    boundingBoxMsgs.x = width;
    boundingBoxMsgs.y = height;
    boundingBoxMsgs.z = depth;
    
    return boundingBoxMsgs;
}

geometry_msgs::Pose toPoseMsgs(PrologBindings bdg)
{
    tf::Quaternion quaternion = prologBindingToQuaternion(bdg);
    tf::Vector3 point = prologBindingToPosition(bdg);

    geometry_msgs::Pose pose;

    pose.position.x = point.getX();
    pose.position.y = point.getY();
    pose.position.z = point.getZ();

    pose.orientation.x = quaternion.getX();
    pose.orientation.y = quaternion.getY();
    pose.orientation.z = quaternion.getZ();
    pose.orientation.w = quaternion.getW();

    return pose;
}

bool get_fixed_kitchen_objects(knowledge_msgs::GetFixedKitchenObjects::Request  &req, knowledge_msgs::GetFixedKitchenObjects::Response &res)
{
    std::string query = createQuery();
    Prolog pl;
  	PrologQueryProxy bdgs = pl.query(query);

    bool succes = false;
    for(PrologQueryProxy::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    {
        succes = true;
        PrologBindings bdg = *it;

        res.names.push_back(bdg["Object"]);
        res.poses.push_back(toPoseMsgs(bdg));
        res.bounding_boxes.push_back(toBoundingBoxMsgs(bdg));
    }

    if(succes)
    {
        res.frame_id = "/map";
    }

    return succes;
}

double prologValueToDouble2(PrologValue value)
{
    double result;

    if(value.type() == PrologValue::value_type::DOUBLE)
    {
        result = value;
    }
    
    if(value.type() == PrologValue::value_type::INT)
    {
        int64_t temp = value;
        result = static_cast<double>(temp);
    }
    return result;

}

tf::Quaternion prologBindingToQuaternion2(PrologBindings bdg)
{
    std::vector<PrologValue> quaternionList = bdg["Quaternion"].as<std::vector<PrologValue>>();

    ROS_INFO_STREAM("1");
    double x = prologValueToDouble2(quaternionList.at(0));
    ROS_INFO_STREAM("2");
    double y = prologValueToDouble2(quaternionList.at(1));
    ROS_INFO_STREAM("3");
    double z = prologValueToDouble2(quaternionList.at(2));
    double w = prologValueToDouble2(quaternionList.at(3));

    return tf::Quaternion(x, y, z, w);
}

tf::Vector3 prologBindingToPosition2(PrologBindings bdg)
{
    std::vector<PrologValue> translationList = bdg["Translation"].as<std::vector<PrologValue>>();

    double x = prologValueToDouble2(translationList.at(0));
    double y = prologValueToDouble2(translationList.at(1));
    double z = prologValueToDouble2(translationList.at(2));

    return tf::Vector3(x, y, z);
}

geometry_msgs::Vector3 toBoundingBoxMsgs2(PrologBindings bdg)
{
    std::vector<PrologValue> boundingBoxList = bdg["BoundingBox"].as<std::vector<PrologValue>>();

    double width = prologValueToDouble2(boundingBoxList.at(0));
    double height = prologValueToDouble2(boundingBoxList.at(1));
    double depth = prologValueToDouble2(boundingBoxList.at(2));


    geometry_msgs::Vector3 boundingBoxMsgs;
    boundingBoxMsgs.x = width;
    boundingBoxMsgs.y = height;
    boundingBoxMsgs.z = depth;
    
    return boundingBoxMsgs;
}

geometry_msgs::Pose toPoseMsgs2(PrologBindings bdg)
{
    tf::Quaternion quaternion = prologBindingToQuaternion2(bdg);
    tf::Vector3 point = prologBindingToPosition2(bdg);

    geometry_msgs::Pose pose;

    pose.position.x = point.getX();
    pose.position.y = point.getY();
    pose.position.z = point.getZ();

    pose.orientation.x = quaternion.getX();
    pose.orientation.y = quaternion.getY();
    pose.orientation.z = quaternion.getZ();
    pose.orientation.w = quaternion.getW();

    return pose;
}

bool get_fixed_kitchen_objects_2(knowledge_msgs::GetFixedKitchenObjects::Request  &req, knowledge_msgs::GetFixedKitchenObjects::Response &res)
{
    std::string query = "getFixedKitchenObjects2(ObjectName, Translation, Quaternion, BoundingBox)";
    Prolog pl;
    PrologQueryProxy bdgs = pl.query(query);

    bool succes = false;
    for(PrologQueryProxy::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    {
        succes = true;
        PrologBindings bdg = *it;

        res.names.push_back(bdg["ObjectName"]);
        res.poses.push_back(toPoseMsgs2(bdg));
        res.bounding_boxes.push_back(toBoundingBoxMsgs2(bdg));
    }

    if(succes)
    {
        res.frame_id = "/map";
    }

    return succes;
}


   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitchen_model_service");
    ros::NodeHandle nh("~");
   
    ros::ServiceServer service = nh.advertiseService("get_fixed_kitchen_objects", get_fixed_kitchen_objects_2);
    ros::spin();
   
    return 0;
}