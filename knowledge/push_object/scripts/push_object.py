#!/usr/bin/env python

import rospy
from json_prolog import json_prolog
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
from geometry_msgs.msg import *
import tf

def gripper_as_string(gripper):
  if(gripper == Gripper.LEFT_GRIPPER):
    return "left_gripper"
  else:
    return "right_gripper"

def object_url_to_object_name(url):
  splitted_url = url.split("#")
  object_identifier = splitted_url[1]
  splitted_object_identifier = object_identifier.split('_')
  return splitted_object_identifier[0]

def create_query(req):
  return "calculate_push_pose(suturo_object:\'" + req.object_label + "\',\'" + req.object_label + "\',X,Y,Z)"

def push_object(req):
  query_result = prolog.once(create_query(req))

  push_pose = PoseStamped()
  push_pose.header.frame_id = "/map"
  push_pose.header.stamp = rospy.Time(0)
  push_pose.pose.position.x = query_result["X"]
  push_pose.pose.position.y = query_result["Y"]
  push_pose.pose.position.z = query_result["Z"]
  push_pose.pose.orientation.x = 0.0
  push_pose.pose.orientation.y = 0.0
  push_pose.pose.orientation.z = 0.0
  push_pose.pose.orientation.w = 1.0

  return PushObjectResponse(push_pose)

if __name__ == '__main__':
    rospy.init_node('push_object')

    prolog = json_prolog.Prolog()
    rospy.Service('/push_object/push', PushObject, push_object)
   
    rospy.spin()