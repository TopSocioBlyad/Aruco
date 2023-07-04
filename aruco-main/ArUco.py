import cv2
from cv2 import aruco
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)
from camera_props import *

cap = cv2.VideoCapture("/dev/video0") # "/dev/video0"
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)
cap.set(cv2.CAP_PROP_FPS, 30)

counter = 0
object_points = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])
try:
    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
        # print(corners)

        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        matrixs = []

        counter = 0

        def matrot(rvec,tvec):
            rotmat = R.from_rotvec(rvec).as_matrix()
            matrix = np.append(rotmat, np.array([tvec]).T, axis=1)
            matrix = np.append(matrix, np.array([[0, 0, 0, 1]]), axis=0)
            return matrix

        if corners:
            for c in corners:
                if 4 in ids:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, 0.1, camera_matrix, distortion_coefficient)
                    image = cv2.drawFrameAxes(frame_markers, camera_matrix, distortion_coefficient, rvec, tvec, 0.1/ 2)
                    matrixs.append(matrot(rvec[0,0], tvec[0,0]))
                    tvec_id4 = tvec
                    print('tvec4',tvec)
                elif 3 in ids :
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, 0.1, camera_matrix, distortion_coefficient)
                    image = cv2.drawFrameAxes(frame_markers, camera_matrix, distortion_coefficient, rvec, tvec, 0.1, 2)
                    matrixs.append(matrot(rvec[0, 0], tvec[0, 0]))
                    tvec_id3 = tvec
                    print('tvec3', tvec)

                counter += 1


            if len(matrixs)==2:
                affins = np.linalg.inv(matrixs[0]) @ matrixs[1]
                print('aff',affins)
                rotate = R.from_matrix(affins[:3,:3]).as_rotvec()
                #print('rot',rotate)
                #print(tvec_id4 - tvec_id3)
                #print(matrixs[0])
                #print(matrixs[:3,:3] @ rotate.T)


        cv2.imshow('frame', frame_markers)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        if key & 0xFF == ord('s'):
            cv2.imwrite(f"latency_test_{counter}.jpg", frame_markers)
    cv2.destroyWindow('frame')
    cap.release()
except KeyboardInterrupt:
    cv2.destroyWindow('frame')
    cap.release()