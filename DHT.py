import Adafruit_DHT as DHT # import the adafruit Library
import time # import the time 
DHT_SENSOR = DHT.DHT22 # if you use dht11 then write dht11
DHT_PIN = 4 #  connect the GPIO4 pin of raspberry pi to out pin of DHT
while True:
    humidity, temperature = DHT.read_retry(DHT_SENSOR, DHT_PIN)
    if humidity is not None and temperature is not None:
        print("Temp={0:0.1f}*C Humidity={1:0.1f}%".format(temperature, humidity))
    else:
        print("Failed to retrieve data from humidity sensor")
