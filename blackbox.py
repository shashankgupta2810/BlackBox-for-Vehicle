# Libraries
from scipy.spatial import distance
from imutils import face_utils
from tempimage import TempImage
import imutils
import argparse
import warnings
import datetime
import dropbox
import json
import time
import dlib
import cv2
import serial
import sys
import RPi.GPIO as GPIO
from time import sleep
import time, math
import Adafruit_DHT
import csv
import pandas as pd
#Define GPIO pins 
GPIO.setmode(GPIO.BCM) 
GPIO.setup(21,GPIO.IN)#flame
input = GPIO.input(21)# define this pin as a input for flame sensor
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)#define pin mode for crash sensor
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)#define pin mode for 2nd crash sensor
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)# define pin mode for reset switch
GPIO.setwarnings(False)
GPIO.setup(26,GPIO.OUT)#define pin mode for buzzer
# define variable value
dist_meas = 0.00 #speed
km_per_hour = 0
rpm = 0
elapse = 0
sensor = 2 #gpio 2 for optocoupler speed measurement
pulse = 0
dht_counter = 0
temperature = 0
tempe = 0
humidity = 0
timeC = 0
start_timer = time.time()
sw1_pressed = "front detected"
sw2_pressed = "back detected"
not_pressed = "not detected"
fd = "flame detected"
fnd = "flame not detected"
face_detected= "Drowsiness Detected"
face_not = " No Drowsiness "
str1 = "";
str2 = "";
str3 = "";
str4 = "";
send_sms = False
csvfile = "bb.csv"
filename = "Drowsy"
NSOR','LONGITUDE','LATITUDE','DROWSINESS']

# filter warnings, load the configuration and initialize the Dropbox
# client
warnings.filterwarnings("ignore")
conf = json.load(open('conf.json'))
client = None

serial_port_open = False
portName = '/dev/ttyUSB0'
try:
   ser = serial.Serial(port=portName)
   serial_port_open = True
except:
   serial_port_open=False

gpgga_info = "$GPGGA,"
GPGGA_buffer = 0
NMEA_buff = 0
lat = 0.0
longi = 0.0
# check to see if the Dropbox should be used
if conf["use_dropbox"]:
    # connect to dropbox and start the session authorization process
    client = dropbox.Dropbox(conf["dropbox_access_token"])
    print("[SUCCESS] dropbox account linked")
    

def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    ear = (A + B) / (2.0 * C)
    return ear

thresh = 0.25
frame_check = 5
detect = dlib.get_frontal_face_detector()
predict = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")# Dat file is the crux of the code

(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]

# initialize the camera and warmup, then initialize the average frmae, last
# uploaded timestamp, and frame motion counter
print("[INFO] warming up...")
time.sleep(conf["camera_warmup_time"])
avg = None
lastUploaded = datetime.datetime.now()
motionCounter = 0
fields = ['Time', 'Temperature', 'Humidity', 'Speed','Flame Sensor','Front Sensor','Back Sensor','Latitude','Longitude','Drowsiness']
#if index == False:
#fields=df.to_csv(csvfile,index=False)
with open(csvfile, "a")as output:
    writer = csv.writer(output, delimiter=",", lineterminator = '\n')
    writer.writerow(fields)
print("CSV file created")
#Define function for GPS
def convert_to_degrees(raw_value):
    decimal_value = raw_value/100.00
    degrees = int(decimal_value)
    mm_mmmm = (decimal_value-int(decimal_value))/0.6
    position = degrees + mm_mmmm
    position = "%.4f" %(position)
    return position

def init_GPIO():                    # initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(sensor,GPIO.IN,GPIO.PUD_UP)
# define the function For GSM
def send_sms():
    port = serial.Serial("/dev/ttyAMA0", baudrate=9600,parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS ,timeout=1)
    port.write(b'AT+CMGS="9XXXXXXXXX"\r\n') # enter mobile number
    rcv = port.read(10)
    #print (rcv)
    time.sleep(0.1)
    port.write(b'Accident of "CAR Number : MH 01 AB 2021" has occurred \n')  # Message
    port.write(str(timeC).encode())
    port.write(b' \n')
    port.write(str(longi).encode())
    port.write(b' \n')
    port.write(str(lat).encode())
    port.write(b' \n')
    port.write(str(tempe).encode())
    port.write(b' \n')
    port.write(str(humidity).encode())
    port.write(b' \n')
    port.write(str(calculate_speed(15)).encode())
    port.write(b' \n')
    port.write(str(str1).encode())
    port.write(b' \n')
    port.write(str(str2).encode())
    port.write(b' \n')
    port.write(str(str3).encode())
    port.write(b' \n')
    port.write(str(str4).encode())
    port.write(b' \n')
    rcv = port.read(10) 
    port.write(b'"\x1A"') # Enable to send SMS
    for i in range(10):
        rcv = port.read(10)
        time.sleep(0.1)
        
    port.write(b'AT+CMGS="9XXXXXXXXX"\r\n') # enter mobile number
    rcv = port.read(10)
    time.sleep(0.1)
    port.write(b'Accident of "CAR Number : MH 01 AB 2021" has occurred \n')  # Message
    port.write(str(timeC).encode())
    port.write(b' \n')
    port.write(str(longi).encode())
    port.write(b' \n')
    port.write(str(lat).encode())
    port.write(b' \n')
    port.write(str(tempe).encode())
    port.write(b' \n')
    port.write(str(humidity).encode())
    port.write(b' \n')
    port.write(str(calculate_speed(15)).encode())
    port.write(b' \n')
    port.write(str(str1).encode())
    port.write(b' \n')
    port.write(str(str2).encode())
    port.write(b' \n')
    port.write(str(str3).encode())
    port.write(b' \n')
    port.write(str(str4).encode())
    port.write(b' \n')
    rcv = port.read(10) 
    port.write(b'"\x1A"') # Enable to send SMS
    for i in range(10):
        rcv = port.read(10)
        time.sleep(0.1)
# define the function for Optocoupler to measure the speed of vehicle
def calculate_elapse(channel):              # callback function
    global pulse, start_timer, elapse
    pulse+=1                                # increase pulse by 1 whenever interrupt occurred
    elapse = time.time() - start_timer      # elapse for every 1 complete rotation made!
    start_timer = time.time()               # let current time equals to start_timer
def calculate_speed(r_cm):
    global pulse,elapse,rpm,dist_km,dist_meas,km_per_sec,km_per_hour
    if elapse !=0:                          # to avoid DivisionByZero error
        rpm = 1/elapse * 60
        circ_cm = (2*math.pi)*r_cm          # calculate wheel circumference in CM
        dist_km = circ_cm/100000            # convert cm to km
        km_per_sec = dist_km / elapse       # calculate KM/sec
        km_per_hour = km_per_sec * 36     # calculate KM/h
        dist_meas = (dist_km*pulse)*1   # measure distance traverse in meter
        km_per_hour ="{:.2f}".format(km_per_hour)
        return km_per_hour
    #limited_float = "{:.2f}".format(a_float)
def init_interrupt():
    GPIO.add_event_detect(sensor, GPIO.FALLING, callback = calculate_elapse, bouncetime = 20)
if __name__ == '__main__':
    init_GPIO()
    init_interrupt()

cap=cv2.VideoCapture(0)
flag=0
 # here is all progarm which is run
while True:
    input_state = GPIO.input(12)
    input_state1 = GPIO.input(16)
    reset_button = GPIO.input(20)
    try:
       ser = serial.Serial(port=portName)
       serial_port_open = True
    except:
       serial_port_open=False
       print(" GPS sensor could not find Network")
    if serial_port_open== True:
        received_data = (str)(ser.readline()) #read NMEA string received
        GPGGA_data_available = received_data.find(gpgga_info)   #check for NMEA GPGGA string                
        if (GPGGA_data_available>0):
            GPGGA_buffer = received_data.split("$GPGGA,",1)[1]  #store data coming after “$GPGGA,” string
            NMEA_buff = (GPGGA_buffer.split(','))
            nmea_time = []
            nmea_latitude = []
            nmea_longitude = []
            nmea_time = NMEA_buff[0]                    #extract time from GPGGA string
            nmea_latitude = NMEA_buff[1]                #extract latitude from GPGGA string
            nmea_longitude = NMEA_buff[3]               #extract longitude from GPGGA string
            print("NMEA Time: ", nmea_time,'\n')
            lat = (float)(nmea_latitude)
            lat = convert_to_degrees(lat)
            longi = (float)(nmea_longitude)
            longi = convert_to_degrees(longi)
            print ("NMEA Latitude:",lat,"NMEA Longitude", longi,'\n')
            
    dht_counter += 1 
    if dht_counter > 10:
        dht_counter = 0
    #print("create CSV file")
        humidity,temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, 4)
        if humidity is not None and temperature is not None:
            humidity = round(humidity,2)
            temparature = round(temperature,2)
            tempe = "{:.2f}".format(temparature)
            print ('Temperature = {0:0.1f}*c Humidity = {1:0.1f}%'.format(temparature,humidity))
            
        else:
            print ('cannot connect to the sensor!')
    print("Speed of the Vechile: ",calculate_speed(15))
    if (input_state == False) or (input_state1 == False):
        counter = 0
        while (counter < 61) and (reset_button == True):
            reset_button = GPIO.input(20)
            counter +=  1
            time.sleep(1)
            print(counter)
            if (counter >= 60) and (reset_button == True):
                print(" Send the message to registered number")
                send_sms()
                counter = 62
            elif (counter < 60) and (reset_button == False):
                print("loop break")
                counter =62
    if input_state == False :
        print('Impact on front side')
        #time.sleep(1)
        str1 = sw1_pressed
    else:
        print('front not Pressed')
        #time.sleep(1)
        str1 = not_pressed
    if input_state1==False :
        print('Impact on back side')
        #time.sleep(1)
        str2 = sw2_pressed
    else:
        print('back not Pressed')
        #time.sleep(1)
        str2 = not_pressed   
    if (GPIO.input(21)):
        print("Flame Detected")
        str3 = fd
    else:
        print('Flame not detected')
        str3 = fnd
    timeC = time.strftime("%I")+':' +time.strftime("%M")+':'+time.strftime("%S")
    print("Time ",timeC)
    # file name to put the data
    filename='bb.csv'
    # all data is add in excel sheet 
    if dht_counter > 4:
        data = ([timeC,tempe,humidity,calculate_speed(15),str1,str2,str3,lat,longi,str4])
        with open(csvfile, "a")as output:
            writer = csv.writer(output, delimiter=",", lineterminator = '\n')
            writer.writerow(data)
# from here open cv program 
    ret, frame=cap.read()
    frame = imutils.resize(frame, width=450)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    subjects = detect(gray, 0)
    timestamp = datetime.datetime.now()
    ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
    for subject in subjects:
        shape = predict(gray, subject)
        shape = face_utils.shape_to_np(shape)#converting to NumPy Array
        leftEye = shape[lStart:lEnd]
        rightEye = shape[rStart:rEnd]
        leftEAR = eye_aspect_ratio(leftEye)
        rightEAR = eye_aspect_ratio(rightEye)
        ear = (leftEAR + rightEAR) / 2.0
        leftEyeHull = cv2.convexHull(leftEye)
        rightEyeHull = cv2.convexHull(rightEye)
        cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
        cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
        if ear < thresh:
            flag += 1
            print (flag)
            if flag >= frame_check:
                flag = 0
                GPIO.output(26,1)
                cv2.putText(frame, "****************ALERT!****************", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, "****************ALERT!****************", (10,325),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                # check to see if dropbox sohuld be used
                if conf["use_dropbox"]:
                    # write the image to temporary file
                    t = TempImage()
                    cv2.imwrite(t.path, frame)
                    # upload the image to Dropbox and cleanup the tempory image
                    print("[UPLOAD] {}".format(ts))
                    path = "/{base_path}/{timestamp}/{filename}{tt1}.jpg".format(
                        base_path=conf["dropbox_base_path"], timestamp=ts, filename=filename, tt1=ts)
                    client.files_upload(open(t.path, "rb").read(), path)
                    t.cleanup()
                #cv2.imwrite("Drowsy.jpg",frame)
                print ("Drowsy")
                str4 = face_detected 
                
        else:
            flag = 0
            str4 = face_not
            GPIO.output(26,0)
        str4 = face_not
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
cv2.destroyAllWindows()
cap.stop()




