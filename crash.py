"""It is program for crash detection like if you use 2 crash sensor in your vehicle first in front and second on back so using raspberry pi you can detect the crash  
"""
import RPi.GPIO as GPIO # import GPIO library
import time # import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP) # #define pin no 12 of raspberry pi
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)#define pin no 16 of raspberry pi

while True:
    input_state = GPIO.input(12)
    input_state1 = GPIO.input(16)
    if input_state == False :
        print('Front side crash')
        time.sleep(1)
    else :
    	print('Crash not detect')
        time.sleep(1)  
    if input_state1==False :
        print('Back side Crash')
         time.sleep(1)
    else :
        print('Crash not detect')
        time.sleep(1)  
