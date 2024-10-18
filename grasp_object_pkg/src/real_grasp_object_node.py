#!/usr/bin/env python3

'''
This script is the main loop of the real-world grasping experiments
'''
import sys
import rospy
import transforms3d
import numpy as np

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from real_gripper_control import GripperControl
from ur_control import UR_robot

def construct_pose(position, angle, frame_id = "base_link") -> geometry_msgs.msg.Pose:

    # pose_under_camera = function(x, y)
    robot_pose = geometry_msgs.msg.Pose()
    robot_pose.position.x = position[0]
    robot_pose.position.y = position[1]
    robot_pose.position.z = position[2]

    ## transform Euler angles into Quternion, angle = (rx, ry, rz)
    quaternion = transforms3d.euler.euler2quat(angle[0], angle[1], angle[2], 'rxyz')

    robot_pose.orientation.w = quaternion[0]
    robot_pose.orientation.x = quaternion[1]
    robot_pose.orientation.y = quaternion[2]
    robot_pose.orientation.z = quaternion[3]

    # print(transforms3d.euler.quat2euler((0, 7.07109031e-01, 7.07104531e-01, 0), 'rxyz'))
    return robot_pose

def construct_pose2(position, quaternion, frame_id = "base_link") -> geometry_msgs.msg.Pose:

    # pose_under_camera = function(x, y)
    robot_pose = geometry_msgs.msg.Pose()
    robot_pose.position.x = position[0]
    robot_pose.position.y = position[1]
    robot_pose.position.z = position[2]

    robot_pose.orientation.w = quaternion[0]
    robot_pose.orientation.x = quaternion[1]
    robot_pose.orientation.y = quaternion[2]
    robot_pose.orientation.z = quaternion[3]
    # print(transforms3d.euler.quat2euler((0, 7.07109031e-01, 7.07104531e-01, 0), 'rxyz'))
    return robot_pose

def rotate_wrist3(degree):
    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[5] = degree/180*np.pi
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.loginfo("Reset robot successfully!")

def moverobot(frame_id = "base_link") -> geometry_msgs.msg.Pose:

    base2baselink =  np.array([[ 1,  0, 0],
                            [0,  1,  0],
                            [0, 0, 1]])
    ## Construct robotiq_controller
    gripper = GripperControl()
    rospy.sleep(1)
    
    ur = UR_robot()
    path = "/catkin_ws/src/grasp_eyeinhand/"
    
    
    # 动作生成   Measing Cylinder
    mc_matrix_g2B = np.load(path+ "images/" + "mc_matrix_g2B.npy")
    mc_pose_grasp = mc_matrix_g2B[0:3,3]
    
    # mc_goal_position = np.array([(mc_pose_grasp[0]-60)/1000,(mc_pose_grasp[1])/1000,(mc_pose_grasp[2]+100)/1000])
    mc_goal_position = np.array([(mc_pose_grasp[0])/1000,(mc_pose_grasp[1])/1000,(mc_pose_grasp[2]+100)/1000])
    mc_goal_rotation = np.load(path + "action/do" + "/current_rotation.npy")
    
    # mc_goal pose
    mc_goal_pose = construct_pose2(mc_goal_position, mc_goal_rotation, frame_id='base_link')

    # mc up pose
    # mc_up_position = np.array([(mc_pose_grasp[0]-60)/1000,(mc_pose_grasp[1])/1000,(mc_pose_grasp[2]+300)/1000])
    mc_up_position = np.array([(mc_pose_grasp[0])/1000,(mc_pose_grasp[1])/1000,(mc_pose_grasp[2]+300)/1000])
    mc_up_pose = construct_pose2(mc_up_position[0:3], mc_goal_rotation, frame_id='base_link')
    
    # 动作生成   Stirring
    st_matrix_g2B = np.load(path+ "images/" + "st_matrix_g2B.npy")
    st_pose_grasp = st_matrix_g2B[0:3,3]
    
    # st_goal_position = np.array([(st_pose_grasp[0]-50)/1000,(st_pose_grasp[1])/1000,(st_pose_grasp[2]+240)/1000])
    st_goal_position = np.array([(st_pose_grasp[0])/1000,(st_pose_grasp[1])/1000,(st_pose_grasp[2]+240)/1000])
    st_goal_rotation = np.load(path + "action/do" + "/current_rotation.npy")
    
    # st_goal pose
    st_goal_pose = construct_pose2(st_goal_position, st_goal_rotation, frame_id='base_link')

    # st up pose
    st_up_position = np.array([(st_pose_grasp[0])/1000,(st_pose_grasp[1])/1000,(st_pose_grasp[2]+300)/1000])
    st_up_pose = construct_pose2(st_up_position[0:3], st_goal_rotation, frame_id='base_link')
    
    
    # 动作生成  Medicine bottle
    mb_matrix_g2B = np.load(path+ "images/" + "mb_matrix_g2B.npy")
    mb_pose_grasp = mb_matrix_g2B[0:3,3]
    
    # mb_goal_position = np.array([(mb_pose_grasp[0]+55)/1000,(mb_pose_grasp[1])/1000,(mb_pose_grasp[2]+20)/1000])
    mb_goal_position = np.array([(mb_pose_grasp[0])/1000,(mb_pose_grasp[1])/1000,(mb_pose_grasp[2]+20)/1000])
    mb_goal_rotation = np.load(path + "action/do" + "/current_rotation.npy")
    
    # mb_goal pose
    mb_goal_pose = construct_pose2(mb_goal_position, mb_goal_rotation, frame_id='base_link')

    # mb up pose
    # mb_up_position = np.array([(mb_pose_grasp[0]+55)/1000,(mb_pose_grasp[1])/1000,(mb_pose_grasp[2]+130)/1000])
    mb_up_position = np.array([(mb_pose_grasp[0])/1000,(mb_pose_grasp[1])/1000,(mb_pose_grasp[2]+130)/1000])
    mb_up_pose = construct_pose2(mb_up_position[0:3], mb_goal_rotation, frame_id='base_link')
    
    
    # 动作执行

    ur.go_to_cartesian_pose(mc_up_pose)    # 量筒上
    ur.go_to_cartesian_pose(mc_goal_pose)  # 量筒上中间
    
    rospy.sleep(1)
    gripper_width = 35
    width = -255/85*gripper_width +255
    gripper.control_gripper(width)          # 夹爪闭合
    rospy.sleep(1)
    
    ur.go_to_cartesian_pose(mc_up_pose)     # 量筒上
    
    ur.go_to_cartesian_pose(st_up_pose)     # 搅拌器上
    ur.go_to_cartesian_pose(st_goal_pose) 
    
    rotate_wrist3(-202.1)                  # 夹爪旋转
    rospy.sleep(2)
    rotate_wrist3(-90.68)                      # 夹爪水平
    rospy.sleep(2)
    
    ur.go_to_cartesian_pose(st_up_pose) 
    ur.go_to_cartesian_pose(mc_up_pose)       # 量筒上
    ur.go_to_cartesian_pose(mc_goal_pose)   # 量筒中间
    
    rospy.sleep(1)
    gripper.control_gripper(0)               # 夹爪开
    rospy.sleep(1)
    
    ur.go_to_cartesian_pose(mc_up_pose)        # 量筒上
    
    ur.go_to_cartesian_pose(mb_up_pose)    # 药瓶上
    ur.go_to_cartesian_pose(mb_goal_pose)  # 药瓶中间
    
    rospy.sleep(1)
    gripper_width = 49
    width = -255/85*gripper_width +255
    gripper.control_gripper(width)          # 夹爪闭合
    rospy.sleep(1)
    
    ur.go_to_cartesian_pose(mb_up_pose)     # 药瓶上
    
    ur.go_to_cartesian_pose(st_goal_pose)     # 搅拌器xia
    
    rotate_wrist3(-202.1)                  # 夹爪旋转
    rospy.sleep(2)
    rotate_wrist3(-90.68)                      # 夹爪水平
    rospy.sleep(2)
    
    ur.go_to_cartesian_pose(mb_up_pose)       # 瓶上
    ur.go_to_cartesian_pose(mb_goal_pose)   # 瓶中间
    
    rospy.sleep(1)
    gripper.control_gripper(0)               # 夹爪开
    rospy.sleep(1)
    
    ur.go_to_cartesian_pose(mb_up_pose)        # 瓶上
    
    rospy.sleep(1)
    
    return True
    
