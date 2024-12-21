import cv2 as cv
import numpy as np
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# termination criteria
images = glob.glob('/home/jaenote/rokey_week7_ws/robot_cams/world_cam_*.jpg')
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
    else:
        print("Not found")

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None) 

img = cv.imread('/home/jaenote/rokey_week7_ws/robot_cams/world_cam_9.jpg')

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

# crop
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv.imshow('Cropped Image', dst)
cv.waitKey(0)
cv.destroyAllWindows()
cv.imwrite('/home/jaenote/rokey_week7_ws/robot_cams/undistorted.jpg', dst)

# 파라미터들 저장 
np.savez('//home/jaenote/rokey_week7_ws/robot_cams/calib.npz', mtx=mtx, dist=dist, newcameramtx=newcameramtx, roi=roi)