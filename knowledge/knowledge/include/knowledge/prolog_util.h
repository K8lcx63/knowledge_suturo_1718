#ifndef PROLOG_UTIL_H
#define PROLOG_UTIL_H
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <json_prolog/prolog.h>
#include <string>

using namespace json_prolog;

class PrologUtil
{
public:
    static std::string toCamelCase(std::string);

    static double prologValueToDouble(PrologValue value);

    static std::string pointToPrologList(geometry_msgs::Point point);

    static geometry_msgs::Point prologBindingToPoint(PrologBindings bdg, std::string field_name);

    static geometry_msgs::PointStamped prologBindingToPoint(PrologBindings bdg, std::string field_name, std::string frame_id);

    static std::string quaternionToPrologList(geometry_msgs::Quaternion quaternion);

    static std::string poseToPrologList(geometry_msgs::Pose pose);
};

#endif