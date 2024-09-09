# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import rospy
import moveit_commander

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

class UR_robot(object):
    """UR_robot moveit control class"""

    def __init__(self):
        super(UR_robot, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def get_current_pose(self):
        # 获取并打印当前的末端执行器位置和姿态
        current_pose = self.move_group.get_current_pose().pose
        rospy.loginfo("Current Pose of the End-Effector: Position - x: {0}, y: {1}, z: {2}; Orientation - x: {3}, y: {4}, z: {5}, w: {6}".format(
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ))
        
        return current_pose
