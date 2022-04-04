
# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

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

def writeNumber(value, offset):
    #bus.write_byte(address, value)
    bus.write_i2c_block_data(address, offset, value)
    return -1

#(state:1byte)(distance:3bytes)(angle:20bytes)
def buildPackage(dist, angle, act):
    pack = []
    
    #Fill pack
    pack[0] = act
    str_dist = str(dist)
    str_ang = str(angle)
    i = 1

    for d in str_dist:
        pack[i]
        i = i + 1

    for d in str_ang:
        pack[i]
        i = i + 1

    #Send the byte package
    writeNumber(pack, 0)


def readNumber(offset):
    number = bus.read_i2c_block_data(address, offset, 32)
    return number