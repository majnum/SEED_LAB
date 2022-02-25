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


def writeNumber(value, offset):
    #bus.write_byte(address, value)
    bus.write_i2c_block_data(address, offset, value)
    return -1

def readNumber(offset):
    number = bus.read_i2c_block_data(address, offset, 32)
    return number

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
    message = "Setpoint:" + str(setpoint) + "\nPosition:" + str(position) 
    lcd.message = message

def nothing(x):
    pass

def imgDisp(imgname, img):
    cv.imshow(imgname, img)
    cv.waitKey(0)
    cv.destroyAllWindows()

##need section turning on picamera, saving awb values, turning off awb
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
    
camera.stop_preview()
avgRedAwb = sum(awbRed)/len(awbRed)
avgBlueAwb = sum(awbBlue)/len(awbBlue)
avgAwb = (avgRedAwb, avgBlueAwb)
camera.awb_mode = 'off'
camera.awb_gains = avgAwb

     
number = readNumber(0); 

##marker detection - use neon pink marker/josh's pencil
while (1):
    camera.capture('markerPic.jpg')
    img = cv.imread('markerPic.jpg', 1)
    #img = cv.resize(img, None, fx = 0.5, fy = 0.5, interpolation = cv.INTER_LINEAR)
    img2 = cv.cvtColor(img, cv.COLOR_BGR2HSV)


    #hsv bounds
    lowerBound = (50, 95, 60)
    upperBound = (85, 255, 255)
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
    combinedImage = np.concatenate((img, closing), axis = 1)

    #finding which region
    #print('the hexagon is located at x = ', avg[1], ', y = ', avg[0])
    #topLeft, topRight, bottomLeft, bottomRight = 0
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
    
    vals = []
    if region != -1:
        vals.append(region)
        writeNumber(vals, 0)
    displayRes(region, "5")
    
    number = readNumber(0)
    message = ""
    for num in number:
        if num != 0:
            message = message + chr(num)
            
    print(message)
    
    time.sleep(2)
    #cv.imshow('combined image', combinedImage)
    #cv.waitKey(3000)
    #cv.destroyAllWindows()
    
    
    #Control update rate for rotary and camera
     
    #for i in range(10):
     #   number = readNumber(0)
      #  message = ""
       # for num in number:
        #       message = message + chr(num)
        #print(message);        
        #displayRes(region, "5")
        #time.sleep(0.05)