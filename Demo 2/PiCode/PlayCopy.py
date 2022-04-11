from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
import glob
import smbus2 as smbus
import board
import math
import serial
import os
#



#Set address
ser = serial.Serial('/dev/ttyACM0', 115200)
#Wait for connection to complete
#time.sleep(1)
#
# Initialise I2C bus.
#i2c = board.I2C()  # uses board.SCL and board.SDA

# for RPI version 1, use “bus = smbus.SMBus(0)”
#bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
#address = 0x04

#function that writes a byte array to the I2C wire

def ReadfromArduino():
    while(ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")
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
    blank1 = [0,0,0]
    blank2 = [0,0,0,0,0,0,0,0,0,0]
    str_dist = str(int(dist))
    str_ang = str(angle)
    
    message = "0" + str(act) + "n" + str_dist + "n" + str_ang + '\n'
    #print(message)
    ser.write(message.encode())
    
#def buildPackage(dist=0, angle=0, act=0):
#    pack = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#    
#    if act != 255:
#        #Fill pack
#        pack[0] = act
#        str_dist = str(dist)
#        str_ang = str(angle)
#        i = 1
#     
#        for d in str_dist[0:3]:
#            if d != '.':
#                pack[i] = ord(d)
#                i = i + 1
#
#        i = 4
#        for d in str_ang[0:20]:
#            pack[i] = ord(d)
#            i = i + 1
#
#        #Send the byte package
#        writeNumber(pack, 0)
#        
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
camera.framerate = 30
os.system('sudo vcdbg set awb_mode 0')
time.sleep(2)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
#good 304 gains: 1.5, 1.2
camera.awb_gains = g
print(float(camera.awb_gains[0]), float(camera.awb_gains[1]))
#camera.awb_gains = g

distanceList = []
distance = []
angle = []
minDistance = 100000
cameraClose = False
endTapeClose = False
stage = 0

lowerBound = (90, 100, 100)
upperBound = (120, 255, 255)

while(1):
    img = np.empty((height * width * 3,), dtype=np.uint8)
    camera.capture(img, 'bgr')
    img = img.reshape((height, width, 3))
    img[0:130, 0:img.shape[1]] = (0, 0, 0)
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
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
    isStartClose = imgThresh[(height-30):height, 0:width]
    isEndClose = imgThresh[0:(height-60), 0:width]
    #camera.start_preview(fullscreen = False, window = (1280, 20, 640, 480))
    
    if np.count_nonzero(isStartClose) is not 0:
        cameraClose = True
        print('wow we\'re close to the tape')
    else:
        cameraClose = False
    
    #flag raised if at end of tape
    if np.count_nonzero(isStartClose) is not 0 and np.count_nonzero(isEndClose) is 0:
        endTapeClose = True
        print('wow the of the tape is close')
    else:
        endTapeClose = False
    
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
        #distance = []
        #angle = []
        #cnt = 0
        stage = 1
        #ang = 0
        buildPackage(0, 0, 1)
         
        time.sleep(1)
        ReadfromArduino()
  
     
    if stage == 1:
         if (distanceToTape < 60) and (distanceToTape > 36):
             print(distanceToTape)
             stage = 2
             buildPackage(distanceToTape,angleX,3)
             #time.sleep(1)
             #ReadfromArduino()
             
    if stage == 2:
        print("Stage 2")
        buildPackage(distanceToTape,angleX,9)
        ReadfromArduino()
        if(cameraClose):
            stage = 3
            buildPackage(22,angleX,9)
             
    if stage == 3:
        print("done")
        
#    if stage == 2:
#        min = 300
#        i = 0
#        ind = 0
#        for d in distance[1:len(distance)]:
#           i = i + 1
#           if d < min:
#               min = d
#               ind = i
#        print(distance[ind])
#        print(angle[ind])
#        print(distance)
#        print(angle)
#        
#        if min != 300:
#            buildPackage(distance[ind],angle[ind],3)
#            stage = 3
#        else:
#            print("didn't see any tape")
#            stage = 5
#    
#    if stage == 3:
#        print("hi")
#        try:
#            if(cameraClose):
#                buildPackage(int(distanceToTape),angleX,10)
#                stage = 4
#            else:    
#                buildPackage(int(distanceToTape),angleX,9)
#        except ValueError:
#            print("nan")
#        print("hi")
        
        
    if stage == 4:
        print("done")
        
    if stage == 5:
        print("IDLE")
        


