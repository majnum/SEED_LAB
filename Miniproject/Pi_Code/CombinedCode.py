#EENG350
#Pi Code for Mini Project
#Authors: Dylan Bott, Joshua Higgins
#Description: This code runs on the Raspberry Pi which is communicating with the Arduino
#    It starts of turning on the camera, taking a few pictures to set the AWB.
#    Then it enters an infinite loop where it continuously takes a picutre
#    And finds the marker we've set up in order to tell the Arduino where to
#    rotate the motor to.
#    It then reads rotary values from the Arduino and displays position and setpoint on LCD.

from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
import smbus2 as smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


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
def displayRes(setpoint, position):
    lcd.clear()
    # Set LCD color to green
    lcd.color = [0, 100, 0]
    message = "Setpoint:" + str(setpoint)[0:5:1] + "\nPosition:" + str(position)[0:5:1] 
    lcd.message = message

def nothing(x):
    pass

#only used for debugging and/or demo purposes
def imgDisp(imgname, img):
    cv.imshow(imgname, img)
    cv.waitKey(0)
    cv.destroyAllWindows()

#initializing camera values
region = -1
camera = PiCamera(resolution = (960, 540), framerate=30)
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



while (1):
    #capturing image to be filtered
    camera.capture('markerPic.jpg')
    img = cv.imread('markerPic.jpg', 1)
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    #hsv bounds for to isolate the color of the marker
    #these bounds are set up to find the neon green color of josh's pencil
    lowerBound = (50, 95, 60)
    upperBound = (85, 255, 255)
    mask = cv.inRange(img2, lowerBound, upperBound)
    imgOut = cv.bitwise_and(img, img, mask = mask)

    #img filtering - blur, open, close to remove any noise
    blur = cv.GaussianBlur(imgOut, (5,5), 0)
    kernel = np.ones((8,8), np.uint8)
    opening = cv.morphologyEx(blur, cv.MORPH_OPEN, kernel)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    #finding center of marker to find region later on
    imgGray = cv.cvtColor(closing, cv.COLOR_BGR2GRAY)
    ret, imgThresh = cv.threshold(imgGray, 40, 255, cv.THRESH_BINARY)
    nonZero = imgThresh.nonzero()
    avg = np.mean(nonZero, axis = 1)
    combinedImage = np.concatenate((img, closing), axis = 1) #for demo
    #imgDisp('combined', combinedImage) #for demo

    #finding which region the marker is in using the resolution of the camera
    if not np.any(nonZero): #no marker
        region = -1
        print('no markers found')
    elif avg[1] < 960/2 and avg[0] < 540/2: #top left
        region = 0
        print('marker in top left')
    elif avg[1] > 960/2 and avg[0] < 540/2: #top right
        region = 1
        print('marker in top right')
    elif avg[1] > 960/2 and avg[0] > 540/2: #bottom right
        region = 2
        print('marker in bottom right')
    elif avg[1] < 960/2 and avg[0] > 540/2: #bottom left
        region = 3
        print('marker in bottom left')
    
    #send values from the camera to the I2C wire
    vals = []
    if region != -1:
        vals.append(region)
        writeNumber(vals, 0)
        region = region * 1.57079632679
    
    #recieve values from the Arduino to display
    number = -1
    try:    
        number = readNumber(0)
    except OSError:
        print("I2C Error")
    
    #Convert the array from the Arduino into a displayable string
    message = ""
    for num in number:
        if num != 0:
            message = message + chr(num)
     
    #Produce and send a message to display on the LCD 
    val = int(message) * 0.00785398163397
    pos = str(val)
    displayRes(region, pos)
    
    time.sleep(0.7)