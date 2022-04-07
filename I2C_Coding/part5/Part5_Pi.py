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
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.color = [0, 0, 0]
lcd.clear()


def displayReset():
    lcd.clear()
    # Set LCD color to green
    lcd.color = [0, 100, 0]

def displayRes(sent, received):
    lcd.clear()
    # Set LCD color to green
    lcd.color = [0, 100, 0]
    message = "Sent:  " + str(sent) + "\nGot:  " + str(received) 
    lcd.message = message
    
    
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def displayMes(val):
    lcd.message = "                                "                       
    lcd.message = val

def writeNumber(value, offset):
    #bus.write_byte(address, value)
    bus.write_i2c_block_data(address, offset, value)
    return -1

def readNumber(offset):
    number = bus.read_i2c_block_data(address, offset, 32)
    return number

displayReset()

while True:

    vals = []
    string = input("Enter a short string: ")
    offset = input("Enter an offset value to write to: ")
    offset_read = input("Enter an offset value to read from: ")
    
    for letter in string:
        vals.append(ord(letter))
        
    offset = int(offset)
    offset_read = int(offset_read)
    number = []
    
    if (not offset) and (not offset_read):
        continue

    try:
        writeNumber(vals, offset)
        # print("RPI: Hi Arduino, I sent you ", value)
        # sleep one second
        time.sleep(1)

        number = readNumber(offset_read)
    except OSError:
        print("I2C Error")
        displayMes("I2C Error")
    
    message = ""
    for num in number:
        if num != 0:
            message = message + chr(num)
    if message != "":
        displayMes("")
    print(message)
    #print("Arduino: Hey RPI, got you this number: ", number)
    #print()
    #displayRes(value, number)