import RPi.GPIO as GPIO # import GPIO library
import time # import time
GPIO.setmode(GPIO.BCM)

GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP) #define pin no 20 of raspberry pi

while True:
    input_state = GPIO.input(20)
    if input_state == False:
        print('Button Pressed')
        time.sleep(0.2)
        
    else:
        print("not pressed")
        time.sleep(0.2)
