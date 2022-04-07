import smbus2 as smbus
import time
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value, offset):
    #bus.write_byte(address, value)
    bus.write_byte_data(address, offset, value)
    return -1

def readNumber(offset):
    number = bus.read_byte_data(address, offset)
    return number

while True:
    #var = input("Enter an integer: ")
    #var = int(var)
    value = input("Enter a value: ")
    offset = input("Enter an offset value to write to: ")
    offset_read = input("Enter an offset value to read from: ")
    value  = int(value)
    offset = int(offset)
    offset_read = int(offset_read)
    
    if (not value) and (not offset) and (not offset_read):
        continue

    writeNumber(value, offset)
    print("RPI: Hi Arduino, I sent you ", value)
    # sleep one second
    time.sleep(1)

    number = readNumber(offset_read)
    print("Arduino: Hey RPI, got you this number: ", number)
    print()