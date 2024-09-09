#!/usr/bin/env python3
import rospy
from detect_pose.srv import * #注意是功能包名.srv
from  detect_pose_node import detect_pose

def handle_detect_pose(req):
    is_success = detect_pose()
    return DetectPoseResponse(is_success)

if __name__ == "__main__":
    rospy.init_node('detect_pose_server')
    server = rospy.Service('detect_pose_service', DetectPose, handle_detect_pose)
    rospy.spin()


