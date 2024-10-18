#!/usr/bin/env python3
import cv2
import numpy as np
import transforms3d as tfs
from ur_get_pose import UR_robot
import geometry_msgs.msg
import rospy
def rodrigues_rotation(r, theta):
    # n旋转轴[3x1]
    # theta为旋转角度
    # 旋转是过原点的，n是旋转轴
    r = np.array(r).reshape(3, 1)
    rx, ry, rz = r[:, 0]
    M = np.array([
        [0, -rz, ry],
        [rz, 0, -rx],
        [-ry, rx, 0]
    ])
    R = np.zeros([3,3])
    R[:3, :3] = np.cos(theta) * np.eye(3) +        \
                (1 - np.cos(theta)) * r @ r.T +    \
                np.sin(theta) * M
    return R

def rodrigues_rotation_vec_to_R(v):
    # r旋转向量[3x1]
    theta = np.linalg.norm(v)
    print(theta)
    r = np.array(v).reshape(3, 1) / theta
    return rodrigues_rotation(r, theta)

def r_t_to_homogeneous_matrix(R, T):
    R1 = np.vstack([R, np.array([0, 0, 0])])
    T1 = np.vstack([T, np.array([1])])
    HomoMtr = np.hstack([R1, T1])
    return HomoMtr

def calc_pose():
    
    # 物体相对于相机坐标
    path = "/catkin_ws/src/grasp_eyeinhand/"

    measuring_cylinder_rvec = np.load(path+ "images/" + "measuring_cylinder_rvec.npy")
    measuring_cylinder_tvec = np.load(path+ "images/" + "measuring_cylinder_tvec.npy").reshape(3,1)
    
    stirrer_rvec = np.load(path+ "images/" + "stirrer_rvec.npy")
    stirrer_tvec = np.load(path+ "images/" + "stirrer_tvec.npy").reshape(3,1)
    
    medicine_bottle_rvec = np.load(path+ "images/" + "medicine_bottle_rvec.npy")
    medicine_bottle_tvec = np.load(path+ "images/" + "medicine_bottle_tvec.npy").reshape(3,1)

    mc_r_matrix_o2C = rodrigues_rotation_vec_to_R(measuring_cylinder_rvec)
    mc_matrix_o2C = r_t_to_homogeneous_matrix(mc_r_matrix_o2C,measuring_cylinder_tvec)
    
    st_r_matrix_o2C = rodrigues_rotation_vec_to_R(stirrer_rvec)
    st_matrix_o2C = r_t_to_homogeneous_matrix(st_r_matrix_o2C,stirrer_tvec)
    
    mb_r_matrix_o2C = rodrigues_rotation_vec_to_R(medicine_bottle_rvec)
    mb_matrix_o2C = r_t_to_homogeneous_matrix(mb_r_matrix_o2C,medicine_bottle_tvec)
    
    
    print("mc_matrix_o2C: ",mc_matrix_o2C)
    print("st_matrix_o2C: ",st_matrix_o2C)
    print("st_matrix_o2C: ",mb_matrix_o2C)
    
    # 手眼矩阵
    matrix_C2H = np.loadtxt(path+ "config/" +"matrix.txt")
    print("matrix_C2H: ",matrix_C2H)
    
    # 机器人末端位姿
    ur = UR_robot()
    current_pose = ur.get_current_pose()

    current_position = np.array([(current_pose.position.x)*1000 ,current_pose.position.y*1000, current_pose.position.z*1000]).reshape(3,1)
    #transforms3d四元数转矩阵函数的四元数格式为(w,x,y,z)
    current_rotation = np.array([current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z])
    
    r_matrix_H2B = tfs.quaternions.quat2mat(current_rotation)
    matrix_H2B = r_t_to_homogeneous_matrix(r_matrix_H2B,current_position)
    
    print("matrix_H2B: ",matrix_H2B)

    
    # matrix_g2o = np.array([[1, 0, 0,0],
    #                         [0, -1, 0, 0],
    #                         [0, 0, -1 ,0],
    #                         [0,0,0,1]])
    matrix_g2o_mc = np.array([[1, 0, 0, 60],
                            [0, 1, 0, 0],
                            [0, 0, 1 ,0],
                            [0,0,0,1]])
    matrix_g2o_st = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1 ,0],
                            [0,0,0,1]])
    matrix_g2o_mb = np.array([[1, 0, 0, -55],
                            [0, 1, 0, 0],
                            [0, 0, 1 ,0],
                            [0,0,0,1]])
    
    mc_matrix_g2B = matrix_H2B @ matrix_C2H @ mc_matrix_o2C @ matrix_g2o_mc
    st_matrix_g2B = matrix_H2B @ matrix_C2H @ st_matrix_o2C @ matrix_g2o_st
    mb_matrix_g2B = matrix_H2B @ matrix_C2H @ mb_matrix_o2C @ matrix_g2o_mb
    
    # matrix_g2B[0][3] =  -matrix_g2B[0][3]
    print("量筒抓取位姿g2B: ",mc_matrix_g2B)
    print("搅拌器抓取位姿g2B: ",st_matrix_g2B)
    print("药瓶抓取位姿g2B: ",mb_matrix_g2B)
    
    
    np.save(path+ "images/" + "mc_matrix_g2B.npy",mc_matrix_g2B)
    np.save(path+ "images/" + "st_matrix_g2B.npy",st_matrix_g2B)
    np.save(path+ "images/" + "mb_matrix_g2B.npy",mb_matrix_g2B)
    
    if  (mc_matrix_g2B is not None) and (st_matrix_g2B is not None) and (mb_matrix_g2B is not None):
        return True
    else:
        return False


if __name__ == '__main__':
    rospy.init_node('calc_pose_node', anonymous=True)
    calc_pose()
    # rospy.spin()
    