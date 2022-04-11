import serial
import time

#Set address
ser = serial.Serial('/dev/ttyACM1', 115200)
#Wait for connection to complete
time.sleep(3)

#Function to read serial
def ReadfromArduino():
    while(ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")

while True:
    #How to send a string
    #value = "hello" + "\n"
    value = input("Please enter a number: ")
    #Remeber to encode the string to bytes
    ser.write(value.encode())

    #wait for Arduino to set up response
    time.sleep(2)
    ReadfromArduino()

