#!/usr/bin/env python

import rospy
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
from geometry_msgs.msg import *


if __name__ == '__main__':
    rospy.init_node('test_client_place_object')
    
    rospy.wait_for_service('/place_object/place')
    try:
        place_service = rospy.ServiceProxy('/place_object/place', PlaceObject)
        gripper = Gripper()
        gripper.gripper = Gripper.LEFT_GRIPPER
        res = place_service(gripper, "/map", 0.4, 0.5)
        print(res.place_pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e