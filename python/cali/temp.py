import numpy as np
import cv2 as cv
import glob
 
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*8,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('/home/jaenote/rokey_week7_ws/cali/test_images/Fisheye1_*.jpg')
 
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (8,6), None)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        # cv.imshow('img', img)
        # cv.waitKey(500)
    else:
        print("Not found")

cv.destroyAllWindows()
# print('objpoints:', objpoints)
# print('imgpoints:', imgpoints)
print("len", objpoints[0].shape, imgpoints[0].shape)
print("len",len(objpoints), len(imgpoints), len(images))

# 파라미터 계산
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None) 

# 파라미터 출력 
# print("mtx", mtx)
# print("dist", dist)
# print("rvecs", rvecs)
# print("tvecs", tvecs)

# undistort
img = cv.imread('/home/jaenote/rokey_week7_ws/cali/test_images/Fisheye1_1.jpg')

h,  w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))


# Undistort the image
dst = cv.undistort(img, mtx, dist, None, newcameramtx)
 
# # crop the image
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]


# Display the original and undistorted images side by side
print(img.shape, dst.shape)
combined = np.hstack((img, dst))
combined = cv.resize(combined, (combined.shape[1] // 2, combined.shape[0] // 2))
cv.imshow('Original and Undistorted Image', combined)
cv.waitKey(0)
cv.destroyAllWindows()

x,y,w,h = roi
dest = dst[y:y+h, x:x+w]
cv.imshow('Cropped Image', dest)
cv.waitKey(0)
cv.destroyAllWindows()
