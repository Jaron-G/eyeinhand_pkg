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
    # rvec = np.array([[[-2.89574422 ,-0.03344724,  0.3793738 ]]] )
    # tvec = np.array([[[  25.62165022, -169.08106376, 1140.32568696]]]).reshape(3,1)
    rvec = np.load(path+ "images/" + "rvecs.npy")
    tvec = np.load(path+ "images/" + "tvecs.npy").reshape(3,1)
    r_matrix_g2C = rodrigues_rotation_vec_to_R(rvec)
    matrix_g2C = r_t_to_homogeneous_matrix(r_matrix_g2C,tvec)
    print("matrix_g2C: ",matrix_g2C)
    
    # 手眼矩阵，将相机与夹爪位置重合
    # matrix_C2H= np.array([[ 1, 0, 0, 0],[ 0, 1 , 0, 0],[0, 0 ,1 , 0],[0, 0 ,0 , 1]])
    matrix_C2H = np.loadtxt(path+ "config/" +"matrix.txt")
    print("matrix_C2H: ",matrix_C2H)
    
    # 机器人末端位姿
    ur = UR_robot()
    current_pose = ur.get_current_pose()
    # current_position = np.array([ 100 , 200, 300]).reshape(3,1)
    # current_rotation = np.array([ 1, 0, 0, 0])

    current_position = np.array([(current_pose.position.x)*1000 ,current_pose.position.y*1000, current_pose.position.z*1000]).reshape(3,1)
    #transforms3d四元数转矩阵函数的四元数格式为(w,x,y,z)
    current_rotation = np.array([current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z])
    
    matrix_g2H =  matrix_C2H @ matrix_g2C 
    print("matrix_g2H: ",matrix_g2H)
    
    r_matrix_H2B = tfs.quaternions.quat2mat(current_rotation)
    matrix_H2B = r_t_to_homogeneous_matrix(r_matrix_H2B,current_position)
    print("matrix_H2B: ",matrix_H2B)

    
    matrix_g2B = matrix_H2B @ matrix_C2H @ matrix_g2C 
    print("matrix_g2B: ",matrix_g2B)
    
    np.save(path+ "images/" + "matrix_g2B.npy",matrix_g2B)
    
    if matrix_g2B is not None:
        return True
    else:
        return False


if __name__ == '__main__':
    rospy.init_node('calc_pose_node', anonymous=True)
    calc_pose()
    # rospy.spin()
    