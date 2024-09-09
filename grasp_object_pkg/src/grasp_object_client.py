#!/usr/bin/env python3
import rospy
from grasp_object.srv  import * #注意是功能包名.srv

def grasp_object_client():
    rospy.wait_for_service('grasp_object_service')
    try:
        grasp_object = rospy.ServiceProxy('grasp_object_service', GraspObject)
        resp1 = grasp_object()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node("grasp_object_client")
    rospy.loginfo("Start grasp_object !")
    response = grasp_object_client()
    rospy.loginfo("grasp_object is OK !")
