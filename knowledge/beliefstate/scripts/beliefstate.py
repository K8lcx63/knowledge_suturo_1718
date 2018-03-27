#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest('json_prolog')

import rospy
from json_prolog import json_prolog
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
from geometry_msgs.msg import *
import re
import tf
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker


def to_underscore(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

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
  return "process_grasp_action(suturo_object:\'" + grasp_object_msg.object_label + "\',suturo_action:\'" + gripper_as_string(grasp_object_msg.gripper.gripper) + "\'," + pose_to_prolog_list(grasp_object_msg.grasp_pose) + ")"

def create_query_for_drop_object(drop_object_msg):
  return "process_drop_action(suturo_action:\'" + gripper_as_string(drop_object_msg.gripper.gripper) + "\')"
        
def create_query_object_attached_to_gripper(gripper):
  return "object_attached_to_gripper(suturo_action:\'" + gripper_as_string(gripper) + "\',ObjectIndividual)"

def object_exists(object_class):
  query_result = prolog.once("object_exists(suturo_object:\'" + object_class + "\')")
  return prolog_query_true(query_result)

def process_perceive_action(perceive_object_msg):
    perceive_object_msg.object_pose.pose.orientation.x = 0.0
    perceive_object_msg.object_pose.pose.orientation.y = 0.0
    perceive_object_msg.object_pose.pose.orientation.z = 0.0
    perceive_object_msg.object_pose.pose.orientation.w = 1.0
    exists = object_exists(perceive_object_msg.object_label)
    prolog.once(create_query_for_perceive_object(perceive_object_msg))
    if not exists:
        spawn_object_frame(perceive_object_msg.object_label, perceive_object_msg.object_pose)
    else:
        update_object_frame(perceive_object_msg.object_label, perceive_object_msg.object_pose)

def process_grasp_action(grasp_object_msg):
  prolog.once(create_query_for_grasp_object(grasp_object_msg))
  if(grasp_object_msg.gripper.gripper == Gripper.LEFT_GRIPPER):
    change_reference_frame(grasp_object_msg.object_label, "l_gripper_led_frame") 
  else:
    change_reference_frame(grasp_object_msg.object_label, "r_gripper_led_frame")

def process_drop_action(drop_object_msg):
  query_result = prolog.once(create_query_object_attached_to_gripper(drop_object_msg.gripper.gripper))
  prolog.once(create_query_for_drop_object(drop_object_msg))
  object_name = object_url_to_object_name(query_result["ObjectIndividual"])
  change_reference_frame(object_name, "map")
  
def gripper_empty(req):
  left_gripper_query_result = prolog.once(create_query_object_attached_to_gripper(Gripper.LEFT_GRIPPER))
  right_gripper_query_result = prolog.once(create_query_object_attached_to_gripper(Gripper.RIGHT_GRIPPER))

  left_gripper_empty = prolog_query_false(left_gripper_query_result)
  right_gripper_empty = prolog_query_false(right_gripper_query_result)

  return EmptyGripperResponse(left_gripper_empty, right_gripper_empty)

def objects_to_pick(req):
    query_result = prolog.once("get_two_objects_on_kitchen_island_counter_with_same_storage_place(Object1, Object2)")
    if(prolog_query_true(query_result)):# two object found with equal storage place
        object_url_1 = query_result["Object1"]
        object_url_2 = query_result["Object2"]

        return ObjectsToPickResponse(object_url_to_object_name(object_url_1), object_url_to_object_name(object_url_2))
    else:
        query_result = prolog.once("get_objects_on_kitchen_island_counter(ObjectList)")
        object_list = query_result["ObjectList"]
        if(len(object_list) == 0):# no object remaining
            return ObjectsToPickResponse("", "")
        else:
            if(len(object_list) == 1):# only one object remaining
                return ObjectsToPickResponse(object_url_to_object_name(object_list[0]), "")
            else:#no objects with same storage place but more than one
                return ObjectsToPickResponse(object_url_to_object_name(object_list[0]), object_url_to_object_name(object_list[1]))

def pose_stamped_to_position_tupel(pose_stamped):
    return (pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)

def pose_stamped_to_quaternion_tupel(pose_stamped):
    return (pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w)

def spawn_object_frame(object_name, object_pose):
    object_frame = to_underscore(object_name)
    source_frame_id = object_pose.header.frame_id
    if source_frame_id.startswith('/'):
        source_frame_id = source_frame_id[1:]

    object_pose.header.frame_id = source_frame_id
    object_pose.header.stamp = rospy.Time(0)
    map_pose = transform_listener.transformPose("map", object_pose)
    #map_pose.pose.orientation.x = 0.0
    #map_pose.pose.orientation.y = 0.0
    #map_pose.pose.orientation.z = 0.0
    #map_pose.pose.orientation.w = 1.0
    map_pose.header.frame_id = "map"
    object_frames[object_frame] = map_pose

    spawn_object_mesh(object_name)

def update_object_frame(object_name, object_pose):
    object_frame = to_underscore(object_name)
    source_frame_id = object_pose.header.frame_id
    if source_frame_id.startswith('/'):
        source_frame_id = source_frame_id[1:]

    object_pose.header.frame_id = source_frame_id
    object_pose.header.stamp = rospy.Time(0)
    map_pose = transform_listener.transformPose("map", object_pose)
    #map_pose.pose.orientation.x = 0.0
    #map_pose.pose.orientation.y = 0.0
    #map_pose.pose.orientation.z = 0.0
    #map_pose.pose.orientation.w = 1.0
    map_pose.header.frame_id = "map"
    object_frames[object_frame] = map_pose

def change_reference_frame(object_name, new_reference_frame_id):
    object_frame = to_underscore(object_name)
    old_frame_pose = object_frames[object_frame]

    new_frame_pose = transform_listener.transformPose(new_reference_frame_id, old_frame_pose)
    object_frames[object_frame] = new_frame_pose

def spawn_object_mesh(object_name):
    object_frame = to_underscore(object_name)

    mesh_marker = Marker()
    mesh_marker.header.frame_id = object_frame
    mesh_marker.header.stamp = rospy.get_rostime()
    mesh_marker.pose.orientation.w = 1
    mesh_marker.ns = object_name
    mesh_marker.id = 1
    mesh_marker.action = Marker.ADD
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    mesh_marker.scale = Vector3(1, 1, 1)
    mesh_marker.frame_locked = True
    mesh_marker.mesh_resource = "package://knowledge_common/meshes/" + object_frame + "/" + object_frame + ".dae";
    mesh_marker.mesh_use_embedded_materials = True

    object_marker.append(mesh_marker)

def object_frame_broadcast(event):
    for frame_id, frame_pose in object_frames.iteritems():
      transform_broadcaster.sendTransform(pose_stamped_to_position_tupel(frame_pose), pose_stamped_to_quaternion_tupel(frame_pose), rospy.Time.now(), frame_id, frame_pose.header.frame_id)

def object_mesh_broadcast(event):
    for marker in object_marker:
      marker.header.stamp = rospy.get_rostime()
      marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('beliefstate')

    prolog = json_prolog.Prolog()

    rospy.Subscriber("/beliefstate/perceive_action", PerceivedObject, process_perceive_action)
    rospy.Subscriber("/beliefstate/grasp_action", GraspObject, process_grasp_action)
    rospy.Subscriber("/beliefstate/drop_action", DropObject, process_drop_action)
    rospy.Service('/beliefstate/gripper_empty', EmptyGripper, gripper_empty)
    rospy.Service('/beliefstate/objects_to_pick', ObjectsToPick, objects_to_pick)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

    object_marker = []
    object_frames = {}
    transform_listener = tf.TransformListener()
    transform_broadcaster = tf.TransformBroadcaster()

    rospy.Timer(rospy.Duration(0.02), object_frame_broadcast)
    rospy.Timer(rospy.Duration(0.02), object_mesh_broadcast)
    rospy.spin()

    # rate = rospy.Rate(50.0)
    # while not rospy.is_shutdown():
    #     for frame_id, frame_pose in object_frames.iteritems():
    #         transform_broadcaster.sendTransform(pose_stamped_to_position_tupel(frame_pose), pose_stamped_to_quaternion_tupel(frame_pose), rospy.Time.now(), frame_id, frame_pose.header.frame_id)
            
    #     for marker in object_marker:
    #         marker.header.stamp = rospy.get_rostime()
    #         marker_pub.publish(marker)

    #     rate.sleep()