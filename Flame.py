import RPi.GPIO as GPIO # import GPIO library
import time # import time
GPIO.setmode(GPIO.BCM) # 
GPIO.setup(21,GPIO.IN) # define pin no 21 of raspberry pi
input = GPIO.input(21) # take the data from flame sensor
while True:
  if (GPIO.input(21)):
    print("Flame Detected")
    time.sleep(2)
  else:
    print("Not Detected")
    time.sleep(2)
    
