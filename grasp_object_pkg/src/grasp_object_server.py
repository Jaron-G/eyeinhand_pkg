#!/usr/bin/env python3
import rospy
from grasp_object.srv import * #注意是功能包名.srv
from  grasp_object_node import moverobot
def handle_grasp_object(req):
    is_success = moverobot()
    return GraspObjectResponse(is_success)

if __name__ == "__main__":
    rospy.init_node('grasp_object_server')
    server = rospy.Service('grasp_object_service', GraspObject, handle_grasp_object)
    rospy.spin()


