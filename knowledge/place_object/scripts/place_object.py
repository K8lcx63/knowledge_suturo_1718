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
  return "calculate_place_pose(suturo_action:\'" + gripper_as_string(req.gripper.gripper) + "\', Z, [QX,QY,QZ,QW])"

def place_object(req):
  query_result = prolog.once(create_query(req))
  z_coordinate = query_result["Z"]

  request_point = PointStamped()
  request_point.header.frame_id = req.frame_id
  request_point.header.stamp = rospy.Time(0)
  request_point.point.x = req.x_coordinate
  request_point.point.y = req.y_coordinate

  map_point = transform_listener.transformPoint("map", request_point)

  place_pose = PoseStamped()
  place_pose.header.frame_id = map_point.header.frame_id
  place_pose.header.stamp = rospy.Time.now()
  place_pose.pose.position.x = map_point.point.x
  place_pose.pose.position.y = map_point.point.y
  place_pose.pose.position.z = z_coordinate
  place_pose.pose.orientation.x = query_result["QX"]
  place_pose.pose.orientation.y = query_result["QY"]
  place_pose.pose.orientation.z = query_result["QZ"]
  place_pose.pose.orientation.w = query_result["QW"]

  return PlaceObjectResponse(place_pose)

if __name__ == '__main__':
    rospy.init_node('place_object')

    prolog = json_prolog.Prolog()
    transform_listener = tf.TransformListener()
    rospy.Service('/place_object/place', PlaceObject, place_object)
   
    rospy.spin()