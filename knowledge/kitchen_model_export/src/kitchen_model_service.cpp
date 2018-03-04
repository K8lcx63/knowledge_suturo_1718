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


/*double prologValueToDouble(PrologValue value)
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

tf::Quaternion prologBindingToQuaternion(PrologBindings bdg)
{
    std::vector<PrologValue> quaternionList = bdg["Quaternion"].as<std::vector<PrologValue>>();

    double x = prologValueToDouble(quaternionList.at(0));
    double y = prologValueToDouble(quaternionList.at(1));
    double z = prologValueToDouble(quaternionList.at(2));
    double w = prologValueToDouble(quaternionList.at(3));

    return tf::Quaternion(x, y, z, w);
}

tf::Vector3 prologBindingToPosition(PrologBindings bdg)
{
    std::vector<PrologValue> translationList = bdg["Translation"].as<std::vector<PrologValue>>();

    double x = prologValueToDouble(translationList.at(0));
    double y = prologValueToDouble(translationList.at(1));
    double z = prologValueToDouble(translationList.at(2));

    return tf::Vector3(x, y, z);
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

geometry_msgs::Vector3 toBoundingBoxMsgs(PrologBindings bdg)
{
    std::vector<PrologValue> boundingBoxList = bdg["BoundingBox"].as<std::vector<PrologValue>>();

    double width = prologValueToDouble(boundingBoxList.at(0));
    double height = prologValueToDouble(boundingBoxList.at(1));
    double depth = prologValueToDouble(boundingBoxList.at(2));


    geometry_msgs::Vector3 boundingBoxMsgs;
    boundingBoxMsgs.x = width;
    boundingBoxMsgs.y = height;
    boundingBoxMsgs.z = depth;
    
    return boundingBoxMsgs;
}*/

bool get_fixed_kitchen_objects(knowledge_msgs::GetFixedKitchenObjects::Request  &req, knowledge_msgs::GetFixedKitchenObjects::Response &res)
{
    std::string query = "get_fixed_kitchen_objects(ObjectName, Path, Frame)";
    Prolog pl;
  	PrologQueryProxy bdgs = pl.query(query);

    for(PrologQueryProxy::iterator it=bdgs.begin(); it != bdgs.end(); it++)
    {
        PrologBindings bdg = *it;

        std::string pathTemp = bdg["Path"].toString();
        std::string path = pathTemp.substr(1, pathTemp.size() - 2);

        std::string frameTemp = bdg["Frame"].toString();
        std::string frame = frameTemp.substr(1, frameTemp.size() - 2);

        res.names.push_back(bdg["ObjectName"]);
        res.meshes.push_back(path);
        res.frames.push_back(frame);
    }

    return true;
}
   
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitchen_model_service");
    ros::NodeHandle nh("~");
   
    ros::ServiceServer service = nh.advertiseService("get_fixed_kitchen_objects", get_fixed_kitchen_objects);
    ros::spin();
   
    return 0;
}