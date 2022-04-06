
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


def readNumber(offset=0):
    number = bus.read_i2c_block_data(address, offset, 32)
    return number


#Initial State Machine!
stage = 0

#IDLE2
if stage == -2:
   Time.sleep(0.1)

#IDLE1
if stage == -1:
   stage = readnumber

#Initialize
if stage == 0:
   distance = []
   angle = []
   cnt = 0
   stage = 1

#Localize
if stage == 1:
   buildPackage(0, 0, 0)
   #Dylan's code goes here (Take vertical line photos and calc dist)

   #End Dylan's code
   distance.append(dist)
   ang = readNumber()
   angle.append(ang)

   if ang == 10.69:
      stage = 2

#Turn to tape and go forward
if stage == 2:
   min = 300
   i = -1
   ind = 0
   for d in distance:
       i = i + 1
       if d < min:
           min = d
           ind = i

   buildPackage(distance[i],angle[i],1)
   stage = -1
   
#Begin feedback cycle between camera and arduino
if stage == 3:
   #Dylan's code here (Wide view providing angle and distance)

   #End Dylan's code
   if dist == Nan:
      stage = 4
   else:
      buildPackage(dist,ang,2)

#Continue when tape becomes not visable (~1ft)
if stage == 4:
   buildPackage(0,0,3)
   

   

   
   
	


   