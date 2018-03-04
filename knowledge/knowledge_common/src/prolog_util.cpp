#include <ros/ros.h>
#include <knowledge_common/prolog_util.h>
#include <sstream>
#include <vector>

std::string PrologUtil::toCamelCase(std::string input)
{
    std::replace(input.begin(), input.end(), '_', ' ');  

    std::stringstream tempStream(input);
    std::stringstream resultSteam;
    std::string temp;
    while (tempStream >> temp)
    {
      temp[0] = toupper(temp[0]);
      resultSteam << temp;
    }
    return resultSteam.str();
}

double PrologUtil::prologValueToDouble(PrologValue value)
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

std::string PrologUtil::quaternionToPrologList(geometry_msgs::Quaternion quaternion)
{
    std::stringstream ss;
    ss << "[" << quaternion.x << ","
              << quaternion.y << "," 
              << quaternion.z << "," 
              << quaternion.w << "]";

    return ss.str();
}

std::string PrologUtil::pointToPrologList(geometry_msgs::Point point)
{
    std::stringstream ss;
    ss << "[" << point.x << ","
              << point.y << "," 
              << point.z << "]";
    return ss.str();
}

geometry_msgs::Point PrologUtil::prologBindingToPoint(PrologBindings bdg, std::string field_name)
{
    std::vector<PrologValue> point_list = bdg[field_name].as<std::vector<PrologValue>>();

    double x = prologValueToDouble(point_list.at(0));
    double y = prologValueToDouble(point_list.at(1));
    double z = prologValueToDouble(point_list.at(2));

    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    return point;
}

geometry_msgs::PointStamped PrologUtil::prologBindingToPoint(PrologBindings bdg, std::string field_name, std::string frame_id)
{
  geometry_msgs::Point point = prologBindingToPoint(bdg, field_name);
  geometry_msgs::PointStamped point_stamped;
  point_stamped.point = point;
  point_stamped.header.frame_id = frame_id;

  return point_stamped;
}

std::string PrologUtil::poseToPrologList(geometry_msgs::Pose pose)
{
  std::stringstream ss;
  ss << "[" << PrologUtil::pointToPrologList(pose.position) << ","
            << PrologUtil::quaternionToPrologList(pose.orientation) << "]";
  return ss.str();
}

geometry_msgs::Quaternion PrologUtil::prologBindingToQuaternion(PrologBindings bdg, std::string field_name)
{
    std::vector<PrologValue> quaternion_list = bdg[field_name].as<std::vector<PrologValue>>();

    double x = prologValueToDouble(quaternion_list.at(0));
    double y = prologValueToDouble(quaternion_list.at(1));
    double z = prologValueToDouble(quaternion_list.at(2));
    double w = prologValueToDouble(quaternion_list.at(3));

    geometry_msgs::Quaternion quaternion;
    quaternion.x = x;
    quaternion.y = y;
    quaternion.z = z;
    quaternion.w = w;

    return quaternion;
}

geometry_msgs::Pose PrologUtil::prologBindingToPose(PrologBindings bdg, std::string position_field_name, std::string rotation_field_name)
{
    geometry_msgs::Pose pose;
    pose.position = PrologUtil::prologBindingToPoint(bdg, position_field_name);
    pose.orientation = PrologUtil::prologBindingToQuaternion(bdg, rotation_field_name);

    return pose;
}