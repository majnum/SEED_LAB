from picamera import PiCamera
import cv2 as cv
import numpy as np
import time

def nothing(x):
    pass

def imgDisp(imgname, img):
    cv.imshow(imgname, img)
    cv.waitKey(0)
    cv.destroyAllWindows()

##need section turning on picamera, saving awb values, turning off awb
camera = PiCamera(resolution = (960, 540), framerate=30)
camera.start_preview()
camera.iso = 400
time.sleep(3)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
#take 3 calibration pictures
awbRed = [0, 0, 0, 0]
awbBlue = [0, 0, 0, 0]
for i in range(4):
    camera.capture('calibrationPic%d.jpg' %i)
    print(camera.awb_gains)
    (awbRed[i], awbBlue[i]) = camera.awb_gains
    time.sleep(1)
    
camera.stop_preview()
avgRedAwb = sum(awbRed)/len(awbRed)
avgBlueAwb = sum(awbBlue)/len(awbBlue)
avgAwb = (avgRedAwb, avgBlueAwb)
camera.awb_mode = 'off'
camera.awb_gains = avgAwb
input('press enter to move on to marker detection')

##marker detection - use neon pink marker/josh's pencil
camera.capture('markerPic.jpg')
img = cv.imread('markerPic.jpg', 1)
#img = cv.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv.INTER_LINEAR)
img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)

#creating trackbars for calibration
#cv.namedWindow('calibration')
#cv.createTrackbar('H lower', 'calibration', 0, 255, nothing)
#cv.createTrackbar('H upper', 'calibration', 255, 255, nothing)
#cv.createTrackbar('S lower', 'calibration', 0, 255, nothing)
#cv.createTrackbar('S upper', 'calibration', 255, 255, nothing)
#cv.createTrackbar('V lower', 'calibration', 0, 255, nothing)
#cv.createTrackbar('V upper', 'calibration', 255, 255, nothing)

#while(1):
#    try:
#        cv.imshow('calibration', imgOut)
#    except:
#        pass
        
    #reading trackbar values
#    hUp = cv.getTrackbarPos('H upper', 'calibration')
#    hLow = cv.getTrackbarPos('H lower', 'calibration')
#    sUp = cv.getTrackbarPos('S upper', 'calibration')
#    sLow = cv.getTrackbarPos('S lower', 'calibration')
#    vUp = cv.getTrackbarPos('V upper', 'calibration')
#    vLow = cv.getTrackbarPos('V lower', 'calibration')
    
    #setting color recognition boundaries from trackbars
#    lowerBound = (hLow, sLow, vLow)
#    upperBound = (hUp, sUp, vUp)
    #creating mask based on bounds
#    maskY = cv.inRange(img2, lowerBound, upperBound)
    #and mask with og image to isolate color range selected
#    imgOut = cv.bitwise_and(img, img, mask = maskY)
    #cv.imwrite('imgOut.jpg', imgOut)
    
#    k = cv.waitKey(1) & 0xFF
#    if k == 27:
#        break
    
#cv.destroyAllWindows()


#hsv bounds
lowerBound = (51, 99, 99)
upperBound = (82, 255, 255)
mask = cv.inRange(img2, lowerBound, upperBound)
imgOut = cv.bitwise_and(img, img, mask = mask)

#img filtering
blur = cv.GaussianBlur(imgOut, (5,5), 0)
kernel = np.ones((8,8), np.uint8)
opening = cv.morphologyEx(blur, cv.MORPH_OPEN, kernel)
closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
#finding center of marker
imgGray = cv.cvtColor(closing, cv.COLOR_BGR2GRAY)
ret, imgThresh = cv.threshold(imgGray, 40, 255, cv.THRESH_BINARY)
nonZero = imgThresh.nonzero()
avg = np.mean(nonZero, axis = 1)

#finding which region
#print('the hexagon is located at x = ', avg[1], ', y = ', avg[0])
#topLeft, topRight, bottomLeft, bottomRight = 0
if not np.any(nonZero):
    noMarker = 1
    print('no markers found')
elif avg[1] < 960/2 and avg[0] < 540/2:
    topLeft = 1
    print('marker in top left')
elif avg[1] > 960/2 and avg[0] < 540/2:
    topRight = 1
    print('marker in top right')
elif avg[1] > 960/2 and avg[0] > 540/2:
    bottomRight = 1
    print('marker in bottom right')
elif avg[1] < 960/2 and avg[0] > 540/2:
    bottomLeft = 1
    print('marker in bottom left')
        
    


    