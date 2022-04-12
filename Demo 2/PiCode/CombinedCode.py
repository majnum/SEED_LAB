##This code initiaizes the camera and LCD systems attached to the Pi and their respective values.
##After taking photos in regular periods, this code filters the immage for the blue tape markings, then determines the angle of those markings.
##These angles are then converted into messages that are sent to the LCD display.
#
#from picamera import PiCamera
#import cv2 as cv
#import numpy as np
#import time
#import glob
#import smbus2 as smbus
#import board
#import math#This code initiaizes the camera and LCD systems attached to the Pi and their respective values.
##After taking photos in regular periods, this code filters the immage for the blue tape markings, then determines the angle of those markings.
##These angles are then converted into messages that are sent to the LCD display.
#
from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
import glob
import smbus2 as smbus
import board
import math
from PIL import Image
#
#
# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04
#
#function that writes a byte array to the I2C wire
def writeNumber(value, offset):
    try:
        bus.write_i2c_block_data(address, offset, value)
    except OSError:
        print("I2C Write Error")
        
    return -1
#
#function that reads a byte array off of the I2C wire
def readNumber(offset=0):
    try:
        number = bus.read_i2c_block_data(address, offset, 32)
    except OSError:
        number = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        print("I2C Read Error")
    return number


def decode(pack):
    ret = 0
    mes = ""
    for val in pack:
        if val != 0:
            mes = mes + chr(val)
    if mes != "":
        try:
            ret = float(mes)
             
        except ValueError:
            ret = 0
            print("VE")

    return ret
    

#(state:1byte)(distance:3bytes)(angle:20bytes)
def buildPackage(dist=0, angle=0, act=0):
    pack = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
    #Fill pack
    pack[0] = act
    str_dist = str(dist)
    str_ang = str(angle)
    i = 1
 
    for d in str_dist[0:3]:
        if d != '.':
            pack[i] = ord(d)
            i = i + 1

    i = 4
    for d in str_ang[0:20]:
        pack[i] = ord(d)
        i = i + 1

    #Send the byte package
    writeNumber(pack, 0)


def nothing(x):
    pass

def imgDisp(imgname, img):
    cv.imshow(imgname, img)
    cv.waitKey(0)
    cv.destroyAllWindows()

    

#set camera to only pick up middle slices - avoids picking up table legs as tape
#everytime blue is found, calculate distance, store in array
#shortest distance will be the actual tape instead of blue desk legs/other noise

camera = PiCamera()
#camera.start_preview(fullscreen = False, window = (1280, 20, 640, 480))

width = 640
height = 480
camera.iso = 200
camera.resolution = (width, height) #be careful changing this, will screw up opencv image
camera.framerate = 24
time.sleep(2)
print('made it to 110')
    
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
camera.awb_mode = 'off'
#time.sleep(3)
camera.awb_gains = (1.5, 1.2)
time.sleep(3)
print(float(camera.awb_gains[0]), float(camera.awb_gains[1]))

#camera.capture('calibrate.jpg')
#img = cv.imread('calibrate.jpg')
##img = cv.resize(img, None, fx=0.5, fy=0.5, interpolation = cv.INTER_LINEAR)
#img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
#cv.namedWindow('yellow')
#cv.createTrackbar('H lower', 'yellow', 20, 255, nothing)
#cv.createTrackbar('H upper', 'yellow', 54, 255, nothing)
#cv.createTrackbar('S lower', 'yellow', 154, 255, nothing)
#cv.createTrackbar('S upper', 'yellow', 253, 255, nothing)
#cv.createTrackbar('V lower', 'yellow', 137, 255, nothing)
#cv.createTrackbar('V upper', 'yellow', 256, 255, nothing)
#
#    #loop for trackbars, escape key breaks loop and moves on
#while(1):
#        #just in case
#    try:
#        cv.imshow('yellow', imgOut)
#    except:
#        pass
#        
#        #reading trackbar values
#    hUp = cv.getTrackbarPos('H upper', 'yellow')
#    hLow = cv.getTrackbarPos('H lower', 'yellow')
#    sUp = cv.getTrackbarPos('S upper', 'yellow')
#    sLow = cv.getTrackbarPos('S lower', 'yellow')
#    vUp = cv.getTrackbarPos('V upper', 'yellow')
#    vLow = cv.getTrackbarPos('V lower', 'yellow')
#
#        #setting color recognition boundaries from trackbars
#    lowerBound = (hLow, sLow, vLow)
#    upperBound = (hUp, sUp, vUp)
#        #creating mask based on bounds
#    mask = cv.inRange(img2, lowerBound, upperBound)
#        #and mask with og image to isolate color range selected
#    imgOut = cv.bitwise_and(img, img, mask = mask)
#        #cv.imwrite('imgYout.jpg', imgYout)
#    
#    k = cv.waitKey(1) & 0xFF
#    if k == 27:
#        break
#    
#cv.destroyAllWindows()

distanceList = []
distance = []
angle = []
minDistance = 100000
cameraClose = False
stage = 1
##find blue tape
while(1):
    img = np.empty((height * width * 3,), dtype=np.uint8)
    camera.capture(img, 'bgr')
    img = img.reshape((height, width, 3))
    
    #set top couple rows of pixels to black to avoid picking up anything not the floor
    img[0:130, 0:img.shape[1]] = (0, 0, 0)
    #cv.imwrite('croptest.jpg', img)
    #print('took pic')
    
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    #HSV bounds to isolate blue tape
    lowerBound = (60, 20, 20)
    upperBound = (120, 255, 255)
    mask = cv.inRange(img2, lowerBound, upperBound)
    imgOut = cv.bitwise_and(img, img, mask = mask)
    
    #img filtering
    blur = cv.GaussianBlur(imgOut, (3,3), 0)
    kernel = np.ones((3,3), np.uint8)
    opening = cv.morphologyEx(blur, cv.MORPH_OPEN, kernel)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    
    #find center of tape
    imgGray = cv.cvtColor(closing, cv.COLOR_BGR2GRAY)
    ret, imgThresh = cv.threshold(imgGray, 40, 255, cv.THRESH_BINARY)
    sideBySide = np.concatenate((img, imgOut, closing), axis=1) #for troubleshooting/calibrating
    #sideBySide = cv.resize(sideBySide, None, fx=0.3, fy=0.3, interpolation = cv.INTER_LINEAR)
    cv.imwrite('prePostFilter.jpg', sideBySide)
    nonZero = imgThresh.nonzero()        
    avg = np.mean(nonZero, axis = 1) #avg[1] = x, avg[0] = y
    #check if there are values at the bottom of cameras view
    #cv.imwrite('imgThreshold.jpg', imgThresh)
    isItClose = imgThresh[(height-30):height, 0:width]
    #camera.start_preview(fullscreen = False, window = (1280, 20, 640, 480))
    #o = camera.add_overlay(closing, layer = 3, alpha = 128, fullscreen = False, window = (1280, 20, 640, 480))
    if np.count_nonzero(isItClose) is not 0:
        cameraClose = True
        print('wow we\'re close to the tape')
    else:
        cameraClose = False
    
    #calculating angle
    xFov = 53.5
    yFov = 41.41    
    imgCenterX = closing.shape[1]/2
    imgCenterY = closing.shape[0]/2    
    centerToCenterX = avg[1] - imgCenterX
    centerToCenterY = avg[0] - imgCenterY #<- since i cropped out top half of image
    angleX = -(xFov / 2) * (centerToCenterX / imgCenterX)
    angleY = (yFov / 2) * (centerToCenterY / imgCenterY) 
    
    #if stage == 1: #xFov will be all janky for sweep state, so avoid x angle calculation
    #    angleX = -1
    
    #calculate approximate distance to tape center
    angleCam = 13 #degrees
    cameraHyp = 6.75 + 1.3 #inches plus fudge
    distanceToTape = math.sin(math.radians(90 - angleY)) * cameraHyp / (math.sin(math.radians(angleY + angleCam)))

    print('X angle: ', angleX)
    print('Y angle: ', angleY)
    print('this distance: ', distanceToTape)
    #print('math.sin(angleY): ', math.sin((angleY)))

    #adding distanceJustFound to list keeping track of all distances found
    #distanceList.append(distanceToTape)
    #finding index of smallest distance
    #minDistanceIndex = distanceList.index(min(distanceList))
    
    #buildPackage(72, 10, 1)

    #print('min distance: ', min(distanceList))
    
    #cv.imshow('sideBySide', sideBySide)
    #cv.waitKey(1000)
    #cv.destroyAllWindows()
    #buildPackage(int(distanceToTape), 0, 0)
    #time.sleep(0.01)
    #Initial State Machine!

    #IDLE3
    if stage == -3:
        print("hi")
        time.sleep(0.1)

    #IDLE2
    if stage == -2:
        readLS = readNumber(0)
        std = decode(readLS)
        if std >= 10:
            stage = 4

    #IDLE1
    if stage == -1:
        readLS = readNumber(0)
        std = decode(readLS)
        print(std)
        if std >= 10:
            stage = 3
            
        #time.sleep(0.1)
   #Initialize
    if stage == 0:
       distance = []
       angle = []
       cnt = 0
       stage = 1
       ang = 0

    #Localize
    if stage == 1:
        #pack = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        #writeNumber(pack, 0)
        #time.sleep(1)
        #time.sleep(1)
        #number = decode(readNumber(0))
        #print(number)
        
        buildPackage(0, 0, 1)
       #Dylan's code goes here (Take vertical line photos and calc dist)
        #camera.resolution = (800, 1944)
        #res = (384, 960)
        #resNp = np.empty((384 * 960 * 3,), dtype = np.uint8)
        #resTuple = (384, 960, 3)
        #camera.resolution = res
        #camera.framerate = 24
       #End Dylan's code
        if distanceToTape > 0:
            distance.append(distanceToTape)
        else:
            distance.append(255)
        readLS = readNumber(0)
        ang = decode(readLS)

        print(ang)
        if ang >= 10:
           stage = 2
           
        else:
            angle.append(ang)

    #Turn to tape and go forward
    if stage == 2:
        min = 300
        i = 0
        ind = 0
        for d in distance[1:len(distance)]:
           i = i + 1
           if d < min:
               min = d
               ind = i
        print(distance)
        print(angle)
        #print(distance[ind])
        #print(angle[ind])
        buildPackage(distance[ind],angle[ind],3)
        time.sleep(10)
        stage = -1
       
    #Begin feedback cycle between camera and arduino

    if stage == 3:
        #buildPackage(distance[ind],angle[ind],2)
        print("here")
        
    if stage == 69:
        buildPackage(distance[ind],angle[ind],2)
        stage = -2
        #if distanceToTape != nan:
            
    if stage == 3:
       #Dylan's code here (Wide view providing angle and distance)
        #res = (2592, 1936)
        #resNp = np.empty((2592 * 1936 * 3,), dtype = np.uint8)
        #resTuple = (2592, 1936, 3)
        #camera.resolution = res
        #camera.framerate = 24
       #End Dylan's code
        #if dist == nan:
        #  stage = 4
        #else:
        buildPackage(distanceToTape,angleX,2)

        #print("here")
    #Continue when tape becomes not visable (~1ft)
    if stage == 5:
       buildPackage(0,0,3)



