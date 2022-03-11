from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
import glob
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus2 as smbus
import board
import math


# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [0, 0, 0]
lcd.clear()

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04


#function that writes a byte array to the I2C wire
def writeNumber(value, offset):
    bus.write_i2c_block_data(address, offset, value)
    return -1

#function that reads a byte array off of the I2C wire
def readNumber(offset):
    number = bus.read_i2c_block_data(address, offset, 32)
    return number

#function resets the LCD display to an initial state
def displayReset():
    lcd.clear()
    # Set LCD color to green
    lcd.color = [0, 100, 0]

#Initially reset display
displayReset()

#Displays setpoint and position on LCD
def displayRes(message):
    lcd.clear()
    # Set LCD color to green
    lcd.color = [0, 100, 0] 
    lcd.message = message


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
camera.iso = 200
time.sleep(2)
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

#camera.capture('calibrate.jpg')
#img = cv.imread('calibrate.jpg')
#img = cv.resize(img, None, fx=0.5, fy=0.5, interpolation = cv.INTER_LINEAR)
#img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
#cv.namedWindow('yellow')
#cv.createTrackbar('H lower', 'yellow', 20, 255, nothing)
#cv.createTrackbar('H upper', 'yellow', 54, 255, nothing)
#cv.createTrackbar('S lower', 'yellow', 154, 255, nothing)
#cv.createTrackbar('S upper', 'yellow', 253, 255, nothing)
#cv.createTrackbar('V lower', 'yellow', 137, 255, nothing)
#cv.createTrackbar('V upper', 'yellow', 256, 255, nothing)

    #loop for trackbars, escape key breaks loop and moves on
#while(1):
#        #just in case
#    try:
#        cv.imshow('yellow', imgOut)
#    except:
#        pass
        
        #reading trackbar values
#    hUp = cv.getTrackbarPos('H upper', 'yellow')
#    hLow = cv.getTrackbarPos('H lower', 'yellow')
#    sUp = cv.getTrackbarPos('S upper', 'yellow')
#    sLow = cv.getTrackbarPos('S lower', 'yellow')
#    vUp = cv.getTrackbarPos('V upper', 'yellow')
#    vLow = cv.getTrackbarPos('V lower', 'yellow')

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
    
#cv.destroyAllWindows()

#find blue tape
while(1):
    camera.capture('pic.jpg')
    img = cv.imread('pic.jpg')
    img = img[int(img.shape[0]/2) : int(img.shape[1])]
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    
    #HSV bounds to isolate blue tape
    lowerBound = (100, 100, 100)
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
    imgCenterY = closing.shape[0]
    centerToCenterX = avg[1] - imgCenterX
    centerToCenterY = avg[0]# - imgCenterY <- since i cropped out top half of image
    angleX = -(xFov / 2) * (centerToCenterX / imgCenterX)
    angleY = (yFov / 2) * (centerToCenterY / imgCenterY) + 2.4 #plus 2.4 to correct any assembly errors
    
    #calculate approximate distance to tape center
    if(angleY > 0):
        distanceToTape = 6.875 / math.tan((abs(angleY)) * math.pi / 180)
    else:
        distanceToTape = -1  #made an oopsie if this happens
    
    
    message = "Angle:" + str(angleX)[0:5:1]
    displayRes(message)
    
    vals = []
    if not math.isnan(angleX):
	for ch in str(angleX)[0:8:1]:
		vals.append(ch)
    	writeNumber(vals, 0)
	   

    print('X angle: ', angleX)
    print('Y angle: ', angleY)
    print('distance: ', distanceToTape)
    
    cv.imshow('sideBySide', sideBySide)
    cv.waitKey(1000)
    cv.destroyAllWindows()
    #time.sleep(0.05)
    
