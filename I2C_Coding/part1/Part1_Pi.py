import smbus2 as smbus
import time
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
    #bus.write_byte(address, value)
    bus.write_byte_data(address, 1, value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    return number

while True:
    var = input("Enter an integer: ")
    var = int(var)
    if (not var) and True:
        continue

    writeNumber(var)
    print("RPI: Hi Arduino, I sent you ", var)
    # sleep one second
    time.sleep(1)

    number = readNumber()
    print("Arduino: Hey RPI, I added 5 to your digit creating", number)
    print()