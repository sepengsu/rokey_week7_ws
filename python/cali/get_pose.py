import cv2 as cv
from cv2 import aruco
import numpy as np
import glob

para_path = '/home/jaenote/rokey_week7_ws/robot_cams/calib.npz'
para = np.load(para_path)

CAMERA_PARAMETERS=para
CAMERA_PARAMETERS = {key: para[key] for key in para}
CAMERA_PARAMETERS['real_size'] = (0.105,0.105)

frame = cv.imread('/home/jaenote/rokey_week7_ws/test_images/robot_0_0_26.jpg')
gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
parameters = aruco.DetectorParameters_create() # 마커를 찾을 때 사용할 파라미터

corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) # 마커 찾기

# 마커의 위치 및 방향을 추정
rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners,0.105, CAMERA_PARAMETERS['mtx'], CAMERA_PARAMETERS['dist'])

# # 마커의 위치 및 방향을 출력

for i in range(len(ids)):
    aruco.drawAxis(frame, CAMERA_PARAMETERS['mtx'], CAMERA_PARAMETERS['dist'], rvecs[i], tvecs[i],0.105) # 축을 그림
    aruco.drawDetectedMarkers(frame, corners) # 마커를 그림
    tvec_text = f"tvecs: {tvecs[i]}"
    cv.putText(frame, tvec_text, (10, 30 * (i + 1)), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv.LINE_AA)
    r_matrix = cv.Rodrigues(rvecs[i])[0]
    print('id:', ids[i], 'rvecs:', rvecs[i], 'tvecs:', tvecs[i])
    # 마커의 위치 및 방향을 출력
    print( 'r_matrix:', r_matrix)
    print('tvecs:', tvecs[i])


cv.imshow('frame', frame)
cv.waitKey(0)
cv.destroyAllWindows()


# R 검증 

image = cv.imread('/home/jaenote/rokey_week7_ws/test_images/test_0_0723.jpg')
unit_x = np.array([[1.0], [0.0], [0.0]])
unit_y = np.array([[0.0], [1.0], [0.0]])
unit_z = np.array([[0.0], [0.0], [1.0]])

aruco.drawAxis(image, CAMERA_PARAMETERS['mtx'], CAMERA_PARAMETERS['dist'], rvecs[i], tvecs[i],0.105) 
R = cv.Rodrigues(rvecs[i])[0]
T = tvecs[i]

x_T = np.dot(R, unit_x)
y_T = np.dot(R, unit_y)
z_T = np.dot(R, unit_z)

axis_size = 100
axis_center = (150, 150)

cv.line(image, axis_center, (int(axis_center[0] + axis_size*x_T[0]), int(axis_center[1] + axis_size*x_T[1])), (0, 0, 255), 2)
cv.line(image, axis_center, (int(axis_center[0] + axis_size*y_T[0]), int(axis_center[1] + axis_size*y_T[1])), (0, 255, 0), 2)
cv.line(image, axis_center, (int(axis_center[0] + axis_size*z_T[0]), int(axis_center[1] + axis_size*z_T[1])), (255, 0, 0), 2)

cv.imshow('frame', image)
cv.waitKey(0)
cv.destroyAllWindows()

