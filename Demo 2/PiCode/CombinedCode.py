#This code initiaizes the camera and LCD systems attached to the Pi and their respective values.
#After taking photos in regular periods, this code filters the immage for the blue tape markings, then determines the angle of those markings.
#These angles are then converted into messages that are sent to the LCD display.

from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
import glob
import smbus2 as smbus
import board
import math


# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

#function that writes a byte array to the I2C wire
def writeNumber(value, offset):
    bus.write_i2c_block_data(address, offset, value)
    return -1

#function that reads a byte array off of the I2C wire
def readNumber(offset=0):
    number = bus.read_i2c_block_data(address, offset, 32)
    return number

#(state:1byte)(distance:3bytes)(angle:20bytes)
def buildPackage(dist=0, angle=0, act=0):
    pack = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
    #Fill pack
    pack[0] = act
    str_dist = str(dist)
    str_ang = str(angle)
    i = 1
 
    for d in str_dist[0:3]:
        pack[i] = int(d)
        i = i + 1

    i = 4
    for d in str_ang[0:20]:
        pack[i] = int(d)
        i = i + 1

    #Send the byte package
    writeNumber(pack, 0)


def nothing(x):
    pass

def imgDisp(imgname, img):
    cv.imshow(imgname, img)
    cv.waitKey(0)
    cv.destroyAllWindows()

stage = 1 #360 degree sweep

if stage == 1:
    camera = PiCamera(resolution = (400, 1080), framerate = 30)
else:
    camera = PiCamera()
    

#set camera to only pick up middle slices - avoids picking up table legs as tape
#everytime blue is found, calculate distance, store in array
#shortest distance will be the actual tape instead of blue desk legs/other noise

camera.start_preview()
camera.iso = 200
time.sleep(2)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
camera.stop_preview()
#take 4 calibration pictures
#awbRed = [0, 0, 0, 0]
#awbBlue = [0, 0, 0, 0]
#for i in range(4):
#    camera.capture('calibrationPic%d.jpg' %i)
#    print(camera.awb_gains)
#    (awbRed[i], awbBlue[i]) = camera.awb_gains
#    time.sleep(0.5)
#camera.awb_mode = 'fluorescent'
    
#averaging and setting the AWB values
#camera.stop_preview()
#avgRedAwb = sum(awbRed)/len(awbRed)
#avgBlueAwb = sum(awbBlue)/len(awbBlue)
#avgAwb = (avgRedAwb, avgBlueAwb)
#camera.awb_mode = 'off'
#camera.awb_gains = avgAwb

distanceList = []
minDistance = 100000


stage = 0
#find blue tape
while(1):
    camera.capture('pic.jpg')
    img = cv.imread('pic.jpg')
    #set top couple rows of pixels to black to avoid picking up anything not the floor
    img[0:260, 0:img.shape[1]] = (0, 0, 0)
    #cv.imwrite('croptest.jpg', img)
    
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    #HSV bounds to isolate blue tape
    lowerBound = (75, 70, 90)
    upperBound = (120, 255, 255)
    mask = cv.inRange(img2, lowerBound, upperBound)
    imgOut = cv.bitwise_and(img, img, mask = mask)
    
    #img filtering
    blur = cv.GaussianBlur(imgOut, (3,3), 0)
    kernel = np.ones((10,10), np.uint8)
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
    centerToCenterY = avg[0] - imgCenterY #<- since i cropped out top half of image
    angleX = -(xFov / 2) * (centerToCenterX / imgCenterX)
    angleY = (yFov / 2) * (centerToCenterY / imgCenterY) #fudge
    
    if stage == 1: #xFov will be all janky for sweep state, so avoid x angle calculation
        angleX = -1
    
    #calculate approximate distance to tape center
    angleCam = 13 #degrees
    cameraHyp = 6.75 + 1.3 #fudge
    distanceToTape = math.sin(math.radians(90 - angleY)) * cameraHyp / (math.sin(math.radians(angleY + angleCam)))
        
    print('Y angle: ', angleY)
    print('this distance: ', distanceToTape)
    #print('math.sin(angleY): ', math.sin((angleY)))

    #adding distanceJustFound to list keeping track of all distances found
    distanceList.append(distanceToTape)
    #finding index of smallest distance
    minDistanceIndex = distanceList.index(min(distanceList))
    
    #buildPackage(72, 10, 1)

    #print('min distance: ', min(distanceList))
    
    #cv.imshow('sideBySide', sideBySide)
    #cv.waitKey(1000)
    #cv.destroyAllWindows()
    #buildPackage(int(distanceToTape), 0, 0)
    #time.sleep(0.01)
    #Initial State Machine!

    #IDLE2
    if stage == -2:
       Time.sleep(0.1)

    #IDLE1
    if stage == -1:
       stage = readnumber

    #Initialize
    if stage == 0:
       distance = []
       angle = []
       cnt = 0
       stage = 1

    #Localize
    if stage == 1:
        buildPackage(72, 10, 1)
       #Dylan's code goes here (Take vertical line photos and calc dist)
        #camera = PiCamera(resolution = (400, 1080))
        camera.resolution = (2592, 1944)
       #End Dylan's code
        distance.append(distanceToTape)
        #ang = readNumber(0)
        #angle.append(ang)

        #if ang == 10.69:
         #  stage = 2

    #Turn to tape and go forward
    if stage == 2:
        min = 300
        i = -1
        ind = 0
        for d in distance:
           i = i + 1
           if d < min:
               min = d
               ind = i

        buildPackage(distance[i],angle[i],1)
        stage = -1
       
    #Begin feedback cycle between camera and arduino
    if stage == 3:
       #Dylan's code here (Wide view providing angle and distance)
        #camera = PiCamera(resolution = (2592, 1944))
        camera.resolution = (2592, 1944)
       #End Dylan's code
        if dist == Nan:
           stage = 4
        else:
           buildPackage(dist,ang,2)

    #Continue when tape becomes not visable (~1ft)
    if stage == 4:
       buildPackage(0,0,3)

    

    
