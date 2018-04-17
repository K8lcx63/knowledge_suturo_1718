#!/usr/bin/env python

import rospy
from knowledge_msgs.msg import *
from knowledge_msgs.srv import * 
from geometry_msgs.msg import *


if __name__ == '__main__':
    rospy.init_node('test_client_push_object')
    
    rospy.wait_for_service('/push_object/push')
    try:
        place_service = rospy.ServiceProxy('/push_object/push', PushObject)
        res = place_service("JaMilch")
        print(res.push_pose)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e