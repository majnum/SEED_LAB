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

#find blue tape
while(1):
    camera.capture('pic.jpg')
    img = cv.imread('pic.jpg')
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    #HSV bounds to isolate blue tape
    lowerBound = (100, 120, 170)
    upperBound = (120, 255, 255)
    mask = cv.inRange(img2, lowerBound, upperBound)
    imgOut = cv.bitwise_and(img, img, mask = mask)
    
    #img filtering
    blur = cv.GaussianBlur(imgOut, (3,3), 0)
    kernel = np.ones((14,14), np.uint8)
    opening = cv.morphologyEx(blur, cv.MORPH_OPEN, kernel)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    sideBySide = np.concatenate((img, imgOut, closing), axis=1) #for troubleshooting/calibrating
    sideBySide = cv.resize(sideBySide, None, fx=0.3, fy=0.3, interpolation = cv.INTER_LINEAR)
    cv.imwrite('prePostFilter.jpg', sideBySide)
    
    #find center of tape
    imgGray = cv.cvtColor(closing, cv.COLOR_BGR2GRAY)
    ret, imgThresh = cv.threshold(imgGray, 40, 255, cv.THRESH_BINARY)
    nonZero = imgThresh.nonzero()
    avg = np.mean(nonZero, axis = 1) #avg[1] = x, avg[0] = y
    
    #calculating angle
    xFov = 53.5
    yFov = 41.41
    
    imgCenterX = closing.shape[1]/2
    imgCenterY = closing.shape[0]/2
    centerToCenterX = avg[1] - imgCenterX
    centerToCenterY = avg[0] - imgCenterY
    angleX = (xFov / 2) * (centerToCenterX / imgCenterX)
    angleY = (yFov / 2) * (centerToCenterY / imgCenterY)
    
    print('X angle: ', angleX)
    print('Y angle: ', angleY)
    
    cv.imshow('sideBySide', sideBySide)
    cv.waitKey(5000)
    cv.destroyAllWindows()
    #time.sleep(0.05)
    
