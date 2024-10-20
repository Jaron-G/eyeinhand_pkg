#!/usr/bin/env python3
import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
def detect_pose():
    marker_size = 60   #mm
    path = "/catkin_ws/src/grasp_eyeinhand/images/"
    image = cv2.imread(path+ "scene.png")

    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    arucoParams = cv2.aruco.DetectorParameters()
    arucodetector = cv2.aruco.ArucoDetector(dictionary= arucoDict, detectorParams=arucoParams)
    (corners, ids, rejected) = arucodetector.detectMarkers(image)
    # cameraMatrix = np.array([762.725, 0, 640.5,
    #                      0, 762.725, 640.5,
    #                      0, 0, 1]).reshape(3,3)
    # cameraMatrix = np.array([913.617065, 0, 960.503906,
    #                         0, 913.455261, 550.489502,
    #                         0, 0, 1]).reshape(3,3)
    cameraMatrix = np.array([609.078, 0, 640.169,
                        0,608.97 , 366.826,
                        0, 0, 1]).reshape(3,3)#720p
    dist = np.array([0.0015398, -0.00872068, 0.000730499, 0.000393782, 0.0000648475])
    distCoeffs = dist[0:5].reshape(1,5)
    
    print(ids)
    print(type(ids))
    print(corners)
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))
        rvecs, tvecs,rej = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs)
        for i in range(len(rvecs)):
            cv2.drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[i, :, :], tvecs[i, :, :], 40)
    print(rvecs)
    # print(tvecs)
        
    # cv2.imshow("detect" , image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plt.imshow(image)   
    plt.show()

    # key = cv2.waitKey(0)
    # if key == ord('q'):         # 按esc键退出
    print(ids)
    print(type(ids))
    if ids is not None:
        ids =ids.tolist()
        print(ids)
        print(type(ids))
        if [2] in ids:
            print(ids.index([2]))
            index_2 = ids.index([2])
            # tvecs[index_2][0][1] = tvecs[index_2][0][1]+60
            np.save(path+ "measuring_cylinder_rvec.npy",rvecs[index_2])
            np.save(path+ "measuring_cylinder_tvec.npy",tvecs[index_2])
            print(rvecs[index_2], tvecs[index_2])
            print(len(rvecs[index_2]))
        if [6] in ids:   
            index_6 = ids.index([6])
            np.save(path+ "stirrer_rvec.npy",rvecs[index_6])
            np.save(path+ "stirrer_tvec.npy",tvecs[index_6])
            print(rvecs[index_6], tvecs[index_6])
            print(len(rvecs[index_6]))
        if [8] in ids:   
            index_8 = ids.index([8])
            # tvecs[index_8][0][1] = tvecs[index_8][0][1]-55
            np.save(path+ "medicine_bottle_rvec.npy",rvecs[index_8])
            np.save(path+ "medicine_bottle_tvec.npy",tvecs[index_8])
            print(rvecs[index_8], tvecs[index_8])
            print(len(rvecs[index_8]))
        return True
    else:
        return False
    
    
if __name__ == '__main__':
    detect_pose()


