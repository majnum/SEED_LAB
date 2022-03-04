from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
import glob

def nothing(x):
    pass

def imgDisp(imgname, img):
    cv.imshow(imgname, img)
    cv.waitKey(0)
    cv.destroyAllWindows()

camera = PiCamera()

#detect and recognize strip of 1" blue and report the angle in degrees
#between the camera axis and the tape. Positive angle when tape is to left of
#camera axis

#find distortion coefficients to calibrate camera distortion
    #f = 3.6mm
    #horz FOV = 53.5 deg
    #vert FOV = 41.41 deg

camera.start_preview()
camera.iso = 400
time.sleep(3)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'

#take 4 calibration pictures
awbRed = [0, 0, 0, 0]
awbBlue = [0, 0, 0, 0]
for i in range(4):
    camera.capture('calibrationPic%d.jpg' %i)
    print(camera.awb_gains)
    (awbRed[i], awbBlue[i]) = camera.awb_gains
    time.sleep(0.5)
    
#avaeraging and setting the AWB values
camera.stop_preview()
avgRedAwb = sum(awbRed)/len(awbRed)
avgBlueAwb = sum(awbBlue)/len(awbBlue)
avgAwb = (avgRedAwb, avgBlueAwb)
camera.awb_mode = 'off'
camera.awb_gains = avgAwb

#fixing distortion
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')
 
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2,ret)
        cv.imshow('img',img)
        cv.waitKey(500)

cv.destroyAllWindows()
