#!/usr/bin/env python3
import rospy
from calc_pose.srv import * #注意是功能包名.srv
from  calc_pose_node import calc_pose

def handle_calc_pose(req):
    is_success = calc_pose()
    return CalcPoseResponse(is_success)

if __name__ == "__main__":
    rospy.init_node('calc_pose_server')
    server = rospy.Service('calc_pose_service', CalcPose, handle_calc_pose)
    rospy.spin()


