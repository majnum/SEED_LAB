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

#function that writes a byte array to the I2C wire
def writeNumber(value, offset):
    try:
        bus.write_i2c_block_data(address, offset, value)
    except OSError:
        print("I2C Write Error")
        
    return -1

#function that reads a byte array off of the I2C wire
def readNumber(offset=0):
    try:
        number = bus.read_i2c_block_data(address, offset, 32)
    except OSError:
        number = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        print("I2C Read Error")
    return number

def buildPackage(dist=0, angle=0, act=0):
    pack = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
    if act != 255:
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
camera.start_preview(fullscreen = False, window = (1280, 20, 640, 480))

width = 640
height = 480
camera.iso = 200
camera.resolution = (width, height) #be careful changing this, will screw up opencv image
camera.framerate = 24
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
#g = camera.awb_gains
#print(float(g[0]), float(g[1]))
camera.awb_mode = 'off'
#time.sleep(3)
camera.awb_gains = (1.5, 1.2)
time.sleep(3)
print(float(camera.awb_gains[0]), float(camera.awb_gains[1]))
#camera.awb_gains = g

distanceList = []
distance = []
angle = []
minDistance = 100000
cameraClose = False
stage = 0

while(1):
    img = np.empty((height * width * 3,), dtype=np.uint8)
    camera.capture(img, 'bgr')
    img = img.reshape((height, width, 3))
    img[0:130, 0:img.shape[1]] = (0, 0, 0)
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
    
    if stage == 0:
        distance = []
        angle = []
        cnt = 0
        stage = 1
        ang = 0
        buildPackage(0, 0, 1)
     
    if stage == 1:
        readLS = readNumber(0)
        ang = decode(readLS)
        
        print(ang)
        
        if(ang > 10):
            stage = 2
            
        elif ang > 0:
            distance.append(distanceToTape)
            angle.append(ang)
            
    if stage == 2:
        min = 300
        i = 0
        ind = 0
        for d in distance[1:len(distance)]:
           i = i + 1
           if d < min:
               min = d
               ind = i
        print(distance[ind])
        print(angle[ind])
        print(distance)
        print(angle)
        
        buildPackage(distance[ind],angle[ind],3)
        stage = 3
    
    if stage == 3:
        print("hi")
        #try:
            #buildPackage(int(distanceToTape),int(angleX),9)
        #except ValueError:
            #print("nan")
        #print("hi")
        


