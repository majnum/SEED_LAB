import smbus2 as smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.color = [0, 0, 0]
#lcd.clear()

def displayRes(sent, received):
    pass
    #lcd.clear()
    # Set LCD color to green
    #lcd.color = [0, 100, 0]
    #message = "Sent:  " + str(sent) + "\nGot:  " + str(received) 
    #lcd.message = message

def displayReset():
    pass
    #lcd.clear()
    # Set LCD color to green
    #lcd.color = [0, 100, 0]

def displayNum(val):
    pass
    #lcd.message = "                                "                       
    #lcd.message = val

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

displayReset()

while True:
    # sleep one second
    time.sleep(1)
    
    number = readNumber(0)
    print(number)
    message = ""
    for num in number:
        if num != 0:
            message = message + chr(num)
            
    #displayNum(message)