#PlayCopy is responsible for recieving information from the camera, performing filtering, 
#and sending the appropriate distance, state, and angle to the arduino through the Serial 

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

#Set address
ser = serial.Serial('/dev/ttyACM0', 115200)

#Read code for recieving info across the Serial
def ReadfromArduino():
    while(ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")



#function that reads a byte array off of the I2C wire
def readNumber(offset=0):
    try:
        number = bus.read_i2c_block_data(address, offset, 32)
    except OSError:
        number = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        print("I2C Read Error")
    return number

#Send angles, states, and distances across the Serial to the Arduino
def buildPackage(dist=0, angle=0, act=0):
    blank1 = [0,0,0]
    blank2 = [0,0,0,0,0,0,0,0,0,0]
    str_dist = str(int(dist))
    str_ang = str(angle)
    
    message = "0" + str(act) + "n" + str_dist + "n" + str_ang + '\n'
    #print(message)
    ser.write(message.encode())

def nothing(x):
    pass

camera = PiCamera()
camera.start_preview(fullscreen = False, window = (1280, 20, 640, 480))
#this resolution processes quickly and is still pretty accurate
width = 640
height = 480
camera.iso = 200 #good for brown 304/305
camera.resolution = (width, height) #be careful changing this, will screw up opencv image
camera.framerate = 30
#os.system('sudo vcdbg set awb_mode 0') #this line will break things
time.sleep(3)
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g
print(float(camera.awb_gains[0]), float(camera.awb_gains[1]))
distanceList = []
distance = []
angle = []
minDistance = 100000
cameraClose = False
endTapeClose = False
stage = 0

#HSV bounds for mask and finding tape
lowerBound = (90, 100, 100)
upperBound = (120, 255, 255)

#main loop controlling Edgar
while(1):
    #capturing directly to an openCV object / numpy array
    img = np.empty((height * width * 3,), dtype=np.uint8)
    camera.capture(img, 'bgr')
    img = img.reshape((height, width, 3))
    img[0:130, 0:img.shape[1]] = (0, 0, 0)
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    #isolating blue
    mask = cv.inRange(img2, lowerBound, upperBound)
    imgOut = cv.bitwise_and(img, img, mask = mask)
    
    #img filtering
    blur = cv.GaussianBlur(imgOut, (3,3), 0)
    kernel = np.ones((4,4), np.uint8)
    opening = cv.morphologyEx(blur, cv.MORPH_OPEN, kernel)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    
    #find center of tape
    imgGray = cv.cvtColor(closing, cv.COLOR_BGR2GRAY)
    ret, imgThresh = cv.threshold(imgGray, 40, 255, cv.THRESH_BINARY)
    sideBySide = np.concatenate((img, imgOut, closing), axis=1) #for troubleshooting/calibrating
    cv.imwrite('prePostFilter.jpg', sideBySide) #very useful for debugging
    nonZero = imgThresh.nonzero()        
    avg = np.mean(nonZero, axis = 1) #avg[1] = x, avg[0] = y
    
    #flags that we can raise for start of tape and end of tape - didn't end up being used for this demo
    isStartClose = imgThresh[(height-30):height, 0:width]
    isEndClose = imgThresh[0:(height-60), 0:width]    
    
    #flag for being close to the start of tape
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
    centerToCenterY = avg[0] - imgCenterY
    angleX = -(xFov / 2) * (centerToCenterX / imgCenterX)
    angleY = (yFov / 2) * (centerToCenterY / imgCenterY) 
    
    #if stage == 1: #xFov will be all janky for sweep state, so avoid x angle calculation
    #    angleX = -1
    
    #calculate approximate distance to tape center
    angleCam = 13 #degrees
    cameraHyp = 6.75 + 1.3 #inches plus fudge
    #weird trig to calculate distance - law of cosines mostly I think
    distanceToTape = math.sin(math.radians(90 - angleY)) * cameraHyp / (math.sin(math.radians(angleY + angleCam)))

    print('X angle: ', angleX)
    print('Y angle: ', angleY)
    print('this distance: ', distanceToTape)
    
    #finite state machine
    #Initialization state
    if stage == 0:
        #distance = []
        #angle = []
        #cnt = 0
        stage = 1
        #ang = 0
        buildPackage(0, 0, 1)
         
     #Find any blue that is within the acceptable distance and tell the arduino to go to it
    if stage == 1:
         if (distanceToTape < 65) and (distanceToTape > 36):
             print(distanceToTape)
             stage = 2
             buildPackage(distanceToTape,angleX,3)

    
    #Update angle and distance values to the Arduino         
    if stage == 2:
        print("Stage 2")
        try:
            buildPackage(distanceToTape,angleX,9)
            ReadfromArduino()
            if(cameraClose):
                stage = 3
                buildPackage(22,angleX,9)
        except ValueError:
            print("nan")
    
    # Idle state         
    if stage == 3:
        print("done")
        
        


