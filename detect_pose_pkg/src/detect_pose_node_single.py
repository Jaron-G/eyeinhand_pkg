#!/usr/bin/env python3
import cv2
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
def detect_pose():
    marker_size = 40   #mm
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
    
    # if ids is not None:
    #     cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))
    #     rvecs, tvecs,rej = cv2.aruco.estimatePoseSingleMarkers(corners[0], marker_size, cameraMatrix, distCoeffs)
    #     for i in range(len(rvecs)):
    #         cv2.drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs, tvecs, 40)
    
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))
        rvecs, tvecs,rej = cv2.aruco.estimatePoseSingleMarkers(corners[0], marker_size, cameraMatrix, distCoeffs)
        for i in range(len(rvecs)):
            cv2.drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs, tvecs, 40)
    
        
    # cv2.imshow("detect" , image)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plt.imshow(image)
    plt.show()

    # key = cv2.waitKey(0)
    # if key == ord('q'):         # 按esc键退出
    if ids is not None:
        print('esc break...')
        # cv2.destroyWindow("detect")
        np.save(path+ "rvecs.npy",rvecs)
        np.save(path+ "tvecs.npy",tvecs)
        print(rvecs, tvecs)
        print(len(rvecs))
        return True
    else:
        return False
    
    
if __name__ == '__main__':
    detect_pose()


