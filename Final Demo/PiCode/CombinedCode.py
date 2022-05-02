############################################################
#This code is responsible for running the raspberry pi - computer vision and communication
#Computer vision runs an infinite loop detecting blue tape and processing through multiple calculations/flags
#Communication runs through serial and does some other magic

#By Dylan Bott & Joshua Higgins - Spring 2022

from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
import glob
import smbus2 as smbus
import board
import math
import serial

#timer
start = time.time()
#Set address
ser = serial.Serial('/dev/ttyACM0', 115200)

#Helper read function for Serial communication
def ReadfromArduino():
    while(ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")            

#Build package function sends state, distance, and angles across Serial communication 
def buildPackage(dist=0, angle=0, act=0):
    blank1 = [0,0,0]
    blank2 = [0,0,0,0,0,0,0,0,0,0]
    try:
        str_dist = str(int(dist))
        str_ang = str(angle)
    except ValueError:
        str_dist = "0"
        str_ang = "0"
        act = 0
    message = "0" + str(act) + "n" + str_dist + "n" + str_ang + '\n'
        
    print(message)
    ser.write(message.encode())
    
#Decode helper function decodes information packages recieved from the Arduino        
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

ninetyComing = False

camera = PiCamera()
camera.start_preview(fullscreen = False, window = (1280, 80, 640, 480)) #useful for seeing what camera is seeing on my screen
#this resolution processes quickly and is still pretty accurate
width = 640
height = 480
camera.iso = 200 #good for brown 304/305
camera.resolution = (width, height) #be careful changing this, will screw up opencv image
camera.framerate = 30
#os.system('sudo vcdbg set awb_mode 0') #this line will break things but is sometimes necessary to put into terminal
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

#counter for how many turns - useful for cross detection implementation
ninetyCounter = 0


#main loop controlling Edgar
while(1):
    #resetting flags
    ninetyComing = False
    crossComing = False
    #capturing directly to an openCV object / numpy array
    img = np.empty((height * width * 3,), dtype=np.uint8)
    camera.capture(img, 'bgr')
    img = img.reshape((height, width, 3))
    img[0:130, 0:img.shape[1]] = (0, 0, 0) #black out top of image to help with environment noise
    img[0:350, (width-100):width] = (0, 0, 0) #black out top right of image to help with environment noise
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    cv.imwrite('test.jpg', img) #for debugging purposes
    
    #isolating blue
    mask = cv.inRange(img2, lowerBound, upperBound)
    imgOut = cv.bitwise_and(img, img, mask = mask)
    
    #img filtering
    blur = cv.GaussianBlur(imgOut, (3,3), 0)
    kernel = np.ones((6,6), np.uint8)
    opening = cv.morphologyEx(blur, cv.MORPH_OPEN, kernel)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    
    #find center of tape - 'center of mass'
    imgGray = cv.cvtColor(closing, cv.COLOR_BGR2GRAY)
    ret, imgThresh = cv.threshold(imgGray, 40, 255, cv.THRESH_BINARY)
    sideBySide = np.concatenate((img, imgOut, closing), axis=1) #for troubleshooting/calibrating
    cv.imwrite('prePostFilter.jpg', sideBySide) #very useful for debugging
    nonZero = imgThresh.nonzero()    
    try:
        avg = np.mean(nonZero, axis = 1) #avg[1] = x, avg[0] = y
        closest = np.amax(nonZero, axis = 1)
        end = np.amin(nonZero, axis = 1)
    except:
        closest = 0
        print('saw nothing')
    
    #flags that we can raise for start of tape and end of tape - these turn to true if tape is detected in respective regions
    isStartClose = imgThresh[(height-30):height, 0:width]
    isEndClose = imgThresh[0:(height-60), 0:width]
    isNinetyComing = imgThresh[350:height, (width-60):width]
    isNinetyComing2 = imgThresh[350:height, (width - 200):width]
    
    #flag for being close to the start of tape
    if np.count_nonzero(isStartClose) is not 0:
        cameraClose = True
        #print('wow we\'re close to the tape')
    else:
        cameraClose = False
    
    #flag raised if at end of tape
    if np.count_nonzero(isStartClose) is not 0 and np.count_nonzero(isEndClose) is 0:
        endTapeClose = True
        #print('wow the of the tape is close')
    else:
        endTapeClose = False

    #calculating angle
    xFov = 53.5
    yFov = 41.41
    angleCam = 13 #degrees
    cameraHyp = 6.75 * 0.96 #inches times fudge 
    imgCenterX = closing.shape[1]/2    
    imgCenterY = closing.shape[0]/2    
    centerToCenterX = avg[1] - imgCenterX    
    centerToCenterY = avg[0] - imgCenterY
    angleX = -(xFov / 2) * (centerToCenterX / imgCenterX)
    angleY = (yFov / 2) * (centerToCenterY / imgCenterY)        
    
    #finding closest part of tape
    distanceToClosest = -1
    if closest is not 0:
        centerToClosestY = closest[0] - imgCenterY
        centerToClosestX = closest[1] - imgCenterX
        angleYclosest = (yFov / 2) * (centerToClosestY / imgCenterY)
        angleXclosest = -(xFov / 2) * (centerToClosestX / imgCenterX)
        distanceToClosest = math.sin(math.radians(90 - angleYclosest)) * cameraHyp / (math.sin(math.radians(angleYclosest + angleCam)))        
        print('distance to closest piece of tape: ', distanceToClosest)
        print('x angle to closest piece of tape: ', angleXclosest)
                    
    #flag for 90 degree right turn coming up and calculating distance to that 90 degree turn
    distanceToNinety = -1
    if np.count_nonzero(isNinetyComing) is not 0:        
        ninetyComing = True                
        nonZeroNinety = imgThresh[0:height, (width-60):width].nonzero()
        avgNinety = np.mean(nonZeroNinety, axis = 1)        
        centerToCenterYninety = avgNinety[0] - imgCenterY
        angleYninety = (yFov / 2) * (centerToCenterYninety / imgCenterY)
        distanceToNinety = math.sin(math.radians(90 - angleYninety)) * cameraHyp / (math.sin(math.radians(angleYninety + angleCam)))
        print('distance to 90 deg turn: ', distanceToNinety)        
    else:
        angleYninetyComing = -1
        
    #flag for 90 degrees if we're expecting the last turn - helps account for gap potentially not raising last flag
    if ninetyCounter >= 3 and np.count_nonzero(isNinetyComing) is not 0:
        ninetyComing = True
        nonZeroNinety2 = imgThresh[0:height, (width-200):width].nonzero()
        avgNinety2 = np.mean(nonZeroNinety2, axis = 1)
        centerToCenterYninety2 = avgNinety2[0] - imgCenterY
        angleYninety2 = (yFov / 2) * (centerToCenterYninety / imgCenterY)
        distanceToNinety2 = math.sin(math.radians(90 - angleYninety2)) * cameraHyp / (math.sin(math.radians(angleYninety2 + angleCam)))
    else:
        angleYninetyComing2 = -1
                                              
        
    #finding distance to end of tape
    distanceToEnd = -1
    try:
        if end[0] > 100:
            centerToEndY = end[0] - imgCenterY
            centerToEndX = end[1] - imgCenterX
            angleYend = (yFov / 2) * (centerToEndY / imgCenterY)
            angleXend = -(xFov / 2) * (centerToEndX / imgCenterX)
            distanceToEnd = math.sin(math.radians(90 - angleYend)) * cameraHyp / (math.sin(math.radians(angleYend + angleCam)))
            print('distane to end: ', distanceToEnd)                    
    except:
        pass
    
    #finding distance to the center of the tape the camera sees
    distanceToTape = math.sin(math.radians(90 - angleY)) * cameraHyp / (math.sin(math.radians(angleY + angleCam)))
    if not np.isnan(distanceToTape):
        print('avg distance: ', distanceToTape)
        print('avg x angle: ', angleX)                       
    
    #cross detection - just sees if the tape is wide enough and checks if we have passed enough 90 degree turns
    nonZeroHeight, nonZeroWidth = np.shape(nonZero) 
    print('ninety counter', ninetyCounter)
    if nonZeroWidth >= 14000 and ninetyCounter >= 4:
        crossComing = True
        print('CROSS SPOTTED')
    else:
        crossComing = False
    
    
    ##############################################
    #finite state machine
    #Initialization step
    if stage == 0:
        stage = 1
        buildPackage(0, 0, 1)
            
    #Go to first piece of close blue tape
    if stage == 1:
         if (distanceToClosest < 18) and (distanceToClosest > 14):
             stage = 2
             try:
                 buildPackage(distanceToClosest,angleXclosest,3)
             except NameError:
                 print("name error")

    
    #Begin continuous angle and distance correction
    if stage == 2:
        #Go to the cross if it is seen and a certain number of turns have occured
        if (crossComing == True) and (time.time() - start > 45) and (ninetyCounter >= 4):
            buildPackage(10,angleX,6)
            stage = 5
        #Execute a right turn 
        if (ninetyComing == True) and (distanceToNinety < 15) and ((time.time() - start) > 20):
            buildPackage(0,0,4)
            stage = 3
            begin = time.time()
        
        #Send current distance and angle values to the Arduino without changing states
        if closest is not 0:
            buildPackage(distanceToTape,angleX,9)
            hold = distanceToNinety

    #Waiting state for the right turn to finish   
    if stage == 3:
    
        #Stop turning and go to the cross when the cross is spotted and enough time has passed
        if (crossComing == True) and (time.time() - start > 45):
            buildPackage(10,angleX,6)
            stage = 5
          
        #Stop turning when a close piece of tape is seen and enough time has passed
        if (distanceToClosest < 18) and ((time.time() - begin) > 3) and (abs(angleXclosest) <= 18):
            buildPackage(distanceToTape,angleXclosest,2)
            ninetyCounter = ninetyCounter + 1
            stage = 2
    
    #Final and end state    
    if stage == 5:
        
        print("done")
        ReadfromArduino()
    


