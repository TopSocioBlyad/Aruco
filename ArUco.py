import cv2
from cv2 import aruco
import time
import numpy as np

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)
from camera_props import *

cap = cv2.VideoCapture("/dev/video4") # "/dev/video0"
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
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


        counter = 0
        tvec_id3 = np.array([[[0,0,0]]])
        tvec_id4 = np.array([[[0,0,0]]])

        if corners:
            for c in corners:
                if ids[counter] == 4:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, 0.1, camera_matrix, distortion_coefficient)
                    image = cv2.drawFrameAxes(frame_markers, camera_matrix, distortion_coefficient, rvec, tvec, 0.1, 2)
                    tvec_id4 = tvec
                elif ids[counter] == 3:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, 0.1, camera_matrix, distortion_coefficient)
                    image = cv2.drawFrameAxes(frame_markers, camera_matrix, distortion_coefficient, rvec, tvec, 0.1, 2)
                    tvec_id3 = tvec

                counter += 1
            print('ID3: ', tvec_id3,'ID4: ', tvec_id4)

            print(tvec_id3 - tvec_id4)
            distance = ((tvec_id4[0][0][0] - tvec_id3[0][0][0])**2 + (tvec_id4[0][0][1] - tvec_id3[0][0][1])**2)**0.5
            print(distance)




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