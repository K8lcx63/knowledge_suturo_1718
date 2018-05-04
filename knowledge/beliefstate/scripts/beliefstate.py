#!/usr/bin/env python

import rospy
from json_prolog import json_prolog
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
from geometry_msgs.msg import *
import tf
from std_msgs.msg import *
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

def is_known_object_label(label):
    labels =  ['TomatoSauceOroDiParma', 'HelaCurryKetchup', 
              'PringlesPaprika', 'PringlesSalt', 
              'JaMilch', 'KoellnMuesliKnusperHonigNuss', 'KellogsToppasMini',
              'CupEcoOrange', 'EdekaRedBowl', 'SiggBottle']
    return label in labels

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

def object_exists(object_label):
    if not is_known_object_label(object_label):
        rospy.logerr("Label: \'" + object_label + "\' is an unknonw label!")
        return False

    try:
        query_result = prolog.once("object_exists(suturo_object:\'" + object_label + "\')")
    except:
        return False
    else:
        return prolog_query_true(query_result)

def process_perceive_action(perceive_object_msg):
    #perceive_object_msg.object_pose.pose.orientation.x = 0.0 
    #perceive_object_msg.object_pose.pose.orientation.y = 0.0 
    #perceive_object_msg.object_pose.pose.orientation.z = 0.0 
    #perceive_object_msg.object_pose.pose.orientation.w = 1.0 

    # perceive_object_msg.object_pose.header.stamp = rospy.Time(0)
    # map_pose = transform_listener.transformPose("map", perceive_object_msg.object_pose)
    # map_pose.header.frame_id = "map"
    # perceive_object_msg.object_pose = map_pose

    # quaternion = (
    # perceive_object_msg.object_pose.pose.orientation.x,
    # perceive_object_msg.object_pose.pose.orientation.y,
    # perceive_object_msg.object_pose.pose.orientation.z,
    # perceive_object_msg.object_pose.pose.orientation.w)

    # euler = tf.transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]+3.14159265359
    # yaw = euler[2]

    # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    # perceive_object_msg.object_pose.pose.orientation.x = quaternion[0] 
    # perceive_object_msg.object_pose.pose.orientation.y = quaternion[1] 
    # perceive_object_msg.object_pose.pose.orientation.z = quaternion[2] 
    # perceive_object_msg.object_pose.pose.orientation.w = quaternion[3] 

    if not is_known_object_label(perceive_object_msg.object_label):
        rospy.logerr("Label: \'" + perceive_object_msg.object_label + "\' is an unknonw label!")
        return

    exists = object_exists(perceive_object_msg.object_label)
    try:
        prolog.once(create_query_for_perceive_object(perceive_object_msg))
    except:
        rospy.logerr("Perceiveaction failed, because the prolog query failed!")
    else:
        if not exists:
            spawn_object_frame(perceive_object_msg.object_label, perceive_object_msg.object_pose)
        else:
            update_object_frame(perceive_object_msg.object_label, perceive_object_msg.object_pose)
        publish_collision_object(perceive_object_msg)

def process_grasp_action(grasp_object_msg):
    #check if its a known label
    if not is_known_object_label(grasp_object_msg.object_label):
        rospy.logerr("Label: \'" + grasp_object_msg.object_label + "\' is an unknonw label!")
        return

    #check if the object already exists
    exists = object_exists(grasp_object_msg.object_label)
    if not exists:
        rospy.logerr("Graspaction failed, because object " + grasp_object_msg.object_label + " is never perceived!")
        return

    try:
        prolog.once(create_query_for_grasp_object(grasp_object_msg))
    except:
        rospy.logerr("Graspaction failed, because the prolog query failed!")
    else:
        if(grasp_object_msg.gripper.gripper == Gripper.LEFT_GRIPPER):
            change_reference_frame(grasp_object_msg.object_label, "l_gripper_led_frame") 
        else:
            change_reference_frame(grasp_object_msg.object_label, "r_gripper_led_frame")

def process_drop_action(drop_object_msg):
    try:
        query_result = prolog.once(create_query_object_attached_to_gripper(drop_object_msg.gripper.gripper))
    except:
        rospy.logerr("Dropaction failed, because the prolog query failed!")
    else:
        if prolog_query_true(query_result):
            try:
                prolog.once(create_query_for_drop_object(drop_object_msg))
            except:
                rospy.logerr("Dropaction failed, because the prolog query failed!")
            else:
                object_name = object_url_to_object_name(query_result["ObjectIndividual"])
                change_reference_frame(object_name, "map")
        else:
            rospy.logerr("Dropaction failed, because no object is attached to the gripper!")
  
def gripper_empty(req):
    try:
        left_gripper_query_result = prolog.once(create_query_object_attached_to_gripper(Gripper.LEFT_GRIPPER))
        right_gripper_query_result = prolog.once(create_query_object_attached_to_gripper(Gripper.RIGHT_GRIPPER))
    except:
        rospy.logerr("EmptyGripperService failed, because the prolog query failed!")
    else:
        left_gripper_empty = prolog_query_false(left_gripper_query_result)
        right_gripper_empty = prolog_query_false(right_gripper_query_result)
        return EmptyGripperResponse(left_gripper_empty, right_gripper_empty)

    return EmptyGripperResponse(False, False)

def object_attached_to_gripper(req):
    try:
        query_result = prolog.once(create_query_object_attached_to_gripper(req.gripper.gripper))
    except:
        rospy.logerr("ObjectAttachedToGripperService failed, because the prolog query failed!")
    else:
        if(prolog_query_true(query_result)):
            object_name = object_url_to_object_name(query_result["ObjectIndividual"])
            return GetObjectAttachedToGripperResponse(object_name)

    return GetObjectAttachedToGripperResponse("")

def objects_to_pick(req):
    try:
        query_result = prolog.once("get_two_objects_on_kitchen_island_counter_with_same_storage_place(Object1, Object2)")
    except:
        rospy.logerr("ObjectsToPick failed!")
    else:
        if(prolog_query_false(query_result)):
            return ObjectsToPickResponse("", "")
        else:
            print(query_result)
            object_name_1 = ""
            object_name_2 = ""
            if("Object1" in query_result and query_result["Object1"] != "\'None\'"):
                object_name_1 = object_url_to_object_name(query_result["Object1"])

            if("Object2" in query_result and query_result["Object2"] != "\'None\'"):
                object_name_2 = object_url_to_object_name(query_result["Object2"])

            return ObjectsToPickResponse(object_name_1, object_name_2) 

def pose_stamped_to_position_tupel(pose_stamped):
    return (pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)

def pose_stamped_to_quaternion_tupel(pose_stamped):
    return (pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w)

def spawn_object_frame(object_frame, object_pose):
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

    spawn_object_mesh(object_frame)

def update_object_frame(object_frame, object_pose):
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

def change_reference_frame(object_frame, new_reference_frame_id):
    old_frame_pose = object_frames[object_frame]

    new_frame_pose = transform_listener.transformPose(new_reference_frame_id, old_frame_pose)
    object_frames[object_frame] = new_frame_pose

def spawn_object_mesh(object_frame):
    mesh_marker = Marker()
    mesh_marker.header.frame_id = object_frame
    mesh_marker.header.stamp = rospy.get_rostime()
    mesh_marker.pose.orientation.w = 1
    mesh_marker.ns = object_frame
    mesh_marker.id = 1
    mesh_marker.action = Marker.ADD
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    mesh_marker.scale = Vector3(1, 1, 1)
    mesh_marker.frame_locked = True
    mesh_marker.mesh_resource = "package://knowledge_common/meshes/" + object_frame + "/" + object_frame + ".dae";
    mesh_marker.mesh_use_embedded_materials = True

    object_marker.append(mesh_marker)

def publish_collision_object(perceive_object_msg):
    try:
        query_result = prolog.once("mesh_path(suturo_object:\'" + perceive_object_msg.object_label + "\', MeshPath)")
        mesh_path = query_result["MeshPath"]
    except:
        rospy.logerr("CollisionObjectPublisher failed, because the prolog query failed!")
    else:
        collision_object_msg = PerceivedObjectBoundingBox()
        collision_object_msg.object_label = perceive_object_msg.object_label
        collision_object_msg.mesh_path = mesh_path[1:-1]
        collision_object_msg.pose = perceive_object_msg.object_pose
        collision_object_publisher.publish(collision_object_msg)

def object_frame_broadcast(event):
    for frame_id, frame_pose in object_frames.iteritems():
        transform_broadcaster.sendTransform(pose_stamped_to_position_tupel(frame_pose), pose_stamped_to_quaternion_tupel(frame_pose), rospy.Time.now(), frame_id, frame_pose.header.frame_id)

def object_mesh_broadcast(event):
    for marker in object_marker:
      marker.header.stamp = rospy.get_rostime()
      marker_pub.publish(marker)

def clear_beliefstate(msg):
    prolog.once("clear")
    del object_marker[:]
    object_frames.clear()

if __name__ == '__main__':
    rospy.init_node('beliefstate')

    prolog = json_prolog.Prolog()

    rospy.Subscriber("/beliefstate/clear", String, clear_beliefstate)
    rospy.Subscriber("/beliefstate/perceive_action", PerceivedObject, process_perceive_action)
    rospy.Subscriber("/beliefstate/grasp_action", GraspObject, process_grasp_action)
    rospy.Subscriber("/beliefstate/drop_action", DropObject, process_drop_action)
    rospy.Service('/beliefstate/gripper_empty', EmptyGripper, gripper_empty)
    rospy.Service('/beliefstate/objects_to_pick', ObjectsToPick, objects_to_pick)
    rospy.Service('/beliefstate/object_attached_to_gripper', GetObjectAttachedToGripper, object_attached_to_gripper)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    collision_object_publisher = rospy.Publisher("perceived_object_bounding_box", PerceivedObjectBoundingBox, queue_size=100)

    object_marker = []
    object_frames = {}
    transform_listener = tf.TransformListener()
    transform_broadcaster = tf.TransformBroadcaster()

    rospy.Timer(rospy.Duration(0.02), object_frame_broadcast)
    rospy.Timer(rospy.Duration(0.02), object_mesh_broadcast)
    rospy.spin()