#!/usr/bin/env python

import rospy
from json_prolog import json_prolog
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
from geometry_msgs.msg import *
import tf
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from std_msgs.msg import *

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

def create_place_query(req):
  return "calculate_place_pose(suturo_action:\'" + gripper_as_string(req.gripper.gripper) + "\', Z, [QX,QY,QZ,QW])"

def create_object_height_query(object_label):
  return "object_height(suturo_object:\'" +object_label + "\', Height)"

def place_object(req):
    query_result = prolog.once(create_place_query(req))
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

    object_attached_to_gripper_service = rospy.ServiceProxy('/beliefstate/object_attached_to_gripper', GetObjectAttachedToGripper)
    response = object_attached_to_gripper_service(req.gripper)
    spawn_object_mesh(place_pose.pose.position.x, place_pose.pose.position.y, response.object_label)

    return PlaceObjectResponse(place_pose)

def spawn_mesh(label, x, y, z):
    mesh_marker = Marker()
    mesh_marker.header.frame_id = "map"
    mesh_marker.header.stamp = rospy.get_rostime()
    mesh_marker.pose.position.x = x
    mesh_marker.pose.position.y = y
    mesh_marker.pose.position.z = z
    mesh_marker.pose.orientation.w = 1
    mesh_marker.ns = label + "_place"
    mesh_marker.id = 1
    #mesh_marker.lifetime = rospy.Duration.from_sec(30.0)
    mesh_marker.action = Marker.ADD
    mesh_marker.type = Marker.MESH_RESOURCE
    mesh_marker.color = ColorRGBA(1.0, 1.0, 1.0, 0.6)
    mesh_marker.scale = Vector3(1, 1, 1)
    mesh_marker.frame_locked = True
    mesh_marker.mesh_resource = "package://knowledge_common/meshes/" + label + "/" + label + ".dae";
    mesh_marker.mesh_use_embedded_materials = True

    marker_pub.publish(mesh_marker)
    object_marker_map[label] = mesh_marker

def spawn_object_mesh(x, y, object_label):
    query_result = prolog.once(create_object_height_query(object_label))
    height = query_result["Height"]

    z = 0.85 + height/2
    spawn_mesh(object_label, x, y, z)

def despawn_object_mesh(object_label):
    if(object_label.data in object_marker_map):
        marker = object_marker_map[object_label.data]
        marker.action = Marker.DELETE
        marker.header.stamp = rospy.get_rostime()
        marker_pub.publish(marker)
        del object_marker_map[object_label.data]

if __name__ == '__main__':
    rospy.init_node('place_object')

    object_marker_map = {}
    prolog = json_prolog.Prolog()
    transform_listener = tf.TransformListener()
    rospy.Service('/place_object/place', PlaceObject, place_object)
    rospy.Subscriber('/place_object/despawn_place_preview', String, despawn_object_mesh)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

    rospy.spin()