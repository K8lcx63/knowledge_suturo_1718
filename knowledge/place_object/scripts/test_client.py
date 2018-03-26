#!/usr/bin/env python

import rospy

import rospy
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
from geometry_msgs.msg import *


if __name__ == '__main__':
    rospy.init_node('test_client_place_object')
    
    rospy.wait_for_service('/place_object/place')
    try:
        place_service = rospy.ServiceProxy('/place_object/place', PlaceObject)
        res = place_service(gripper=Gripper.LEFT_GRIPPER, frame_id="/map", x_coordinate=0.5, y_coordinate=0.5)
        print(res.place_pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e