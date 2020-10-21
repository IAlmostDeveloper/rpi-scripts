import adafruit_dht
import board
import RPi.GPIO as GPIO
import time
DHT11_pin = board.D19

def get_temp_hum(pin):
 global humidity, temperature
 temp_sensor = adafruit_dht.DHT11(pin)
 while True:
  temperature = temp_sensor.temperature
  humidity = temp_sensor.humidity
  #humidity, temperature = Adafruit_DHT.read_retry(sensor=temp_sensor, pin = pin, delay_seconds=1)
  print(humidity, " ", temperature)
  time.sleep(1)
  

get_temp_hum(DHT11_pin)
