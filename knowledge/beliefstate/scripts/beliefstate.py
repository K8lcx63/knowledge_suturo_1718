#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
import geometry_msgs.msg

def object_url_to_object_name(url):
  splitted_url = url.split("#")
  object_identifier = splitted_url[1]
  splitted_object_identifier = object_identifier.split('_')
  return splitted_object_identifier[0]

def prolog_query_false(query_result):
  return isinstance(query_result, list)

def prolog_query_true(query_result):
  return isinstance(query_result, dict)

def point_to_prolog_list(point):
  return "[" + str(point.x) + "," + str(point.y) + "," + str(point.z) + "]"

def quaternion_to_prolog_list(quaternion):
  return "[" + str(quaternion.x) + "," + str(quaternion.y) + "," + str(quaternion.z) + "," + str(quaternion.w) + "]"

def pose_to_prolog_list(pose):
  return "[" + point_to_prolog_list(pose.pose.position) + "," + quaternion_to_prolog_list(pose.pose.orientation) + "]"

def gripper_as_string(gripper):
  if(gripper == Gripper.LEFT_GRIPPER):
    return "left_gripper"
  else:
    return "right_gripper"

def create_query_for_perceive_object(perceive_object_msg):
  return "process_perceive_action(suturo_object:\'" + perceive_object_msg.object_label + "\',"+ pose_to_prolog_list(perceive_object_msg.object_pose) + ",\'" + perceive_object_msg.object_pose.header.frame_id +"\')"

def create_query_for_grasp_object(grasp_object_msg):
  return "process_grasp_action(suturo_object:\'" + grasp_object_msg.object_label + "\',suturo_action:\'" + gripper_as_string(grasp_object_msg.gripper.gripper) + "\')"

def create_query_for_drop_object(drop_object_msg):
  return "process_drop_action(suturo_action:\'" + gripper_as_string(drop_object_msg.gripper.gripper) + "\')"
        
def create_query_for_empty_gripper(gripper):
  return "object_attached_to_gripper(suturo_action:\'" + gripper_as_string(gripper) + "\',ObjectIndividual)"

def process_perceive_action(perceive_object_msg):
  prolog.once(create_query_for_perceive_object(perceive_object_msg))

def process_grasp_action(grasp_object_msg):
  prolog.once(create_query_for_grasp_object(grasp_object_msg))

def process_drop_action(drop_object_msg):
  prolog.once(create_query_for_drop_object(drop_object_msg))

def gripper_empty(req):
  left_gripper_query_result = prolog.once(create_query_for_empty_gripper(Gripper.LEFT_GRIPPER))
  right_gripper_query_result = prolog.once(create_query_for_empty_gripper(Gripper.RIGHT_GRIPPER))

  left_gripper_empty = prolog_query_false(left_gripper_query_result)
  right_gripper_empty = prolog_query_false(right_gripper_query_result)

  return EmptyGripperResponse(left_gripper_empty, right_gripper_empty)

def objects_to_pick(req):
  query_result = prolog.once("get_two_objects_on_kitchen_island_counter_with_same_storage_place(Object1, Object2)")
  if(prolog_query_true(query_result)):
    object_url_1 = query_result["Object1"]
    object_url_2 = query_result["Object2"]

    return ObjectsToPickResponse(object_url_to_object_name(object_url_1), object_url_to_object_name(object_url_2))
  else:
    query_result = prolog.once("get_objects_on_kitchen_island_counter(ObjectList)")
    object_list = query_result["ObjectList"]
    if(len(object_list) == 0):
      return ObjectsToPickResponse("", "")
    else:
      if(len(object_list) == 1):
        return ObjectsToPickResponse(object_url_to_object_name(object_list[0]), "")
      else:
        return ObjectsToPickResponse(object_url_to_object_name(object_list[0]), object_url_to_object_name(object_list[1]))


if __name__ == '__main__':
  rospy.init_node('beliefstate')

  prolog = json_prolog.Prolog()

  rospy.Subscriber("/beliefstate/perceive_action", PerceivedObject, process_perceive_action)
  rospy.Subscriber("/beliefstate/grasp_action", GraspObject, process_grasp_action)
  rospy.Subscriber("/beliefstate/drop_action", DropObject, process_drop_action)
  rospy.Service('/beliefstate/gripper_empty', EmptyGripper, gripper_empty)
  rospy.Service('/beliefstate/objects_to_pick', ObjectsToPick, objects_to_pick)

  rospy.spin()