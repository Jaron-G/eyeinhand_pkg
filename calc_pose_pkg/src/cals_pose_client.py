#!/usr/bin/env python3
import rospy
from calc_pose.srv  import * #注意是功能包名.srv

def calc_pose_client():
    rospy.wait_for_service('calc_pose_service')
    try:
        calc_pose = rospy.ServiceProxy('calc_pose_service', CalcPose)
        resp1 = calc_pose()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node("calc_pose_client")
    rospy.loginfo("Start calc_pose !")
    response = calc_pose_client()
    rospy.loginfo("calc_pose is OK !")
