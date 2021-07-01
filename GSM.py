import serial
import RPi.GPIO as GPIO      
import time
GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/ttyAMA0", baudrate=9600,parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS ,timeout=1)

port.write(b'AT+CMGS="91XXXXXXXX"\r\n') # enter your Phone number to send the message
rcv = port.read(10)
print (rcv)
time.sleep(1)
port.write(b'Hey dude meassage is reached to specify number\r\n')  # Message 
rcv = port.read(10)
print (rcv) 
port.write(b'"\x1A"') # Enable to send SMS
for i in range(10):
    rcv = port.read(10)
    print (rcv)
    time.sleep(0)
#if you want send on 2 mobile number uncomment bellow code 
"""
port.write(b'AT+CMGS="91XXXXXXXX"\r\n') # enter your Phone number to send the message
rcv = port.read(10)
print (rcv)
time.sleep(1)
port.write(b'Hey dude meassage is reached to specify number\r\n') 
rcv = port.read(10)
print (rcv) 
port.write(b'"\x1A"') # Enable to send SMS
for i in range(10):
    rcv = port.read(10)"""
