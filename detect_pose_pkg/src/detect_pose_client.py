#!/usr/bin/env python3
import rospy
from detect_pose.srv  import * #注意是功能包名.srv

def detect_pose_client():
    print(1111)
    rospy.wait_for_service('detect_pose_service')
    print(2222)
    try:
        detect_pose = rospy.ServiceProxy('detect_pose_service', DetectPose)
        print(3333)
        resp1 = detect_pose()
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node("detect_pose_client")
    rospy.loginfo("Start detect_pose !")
    response = detect_pose_client()
    rospy.loginfo("detect pose is OK !")
