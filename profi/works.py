# -*- coding: utf-8 -*-
from time import sleep, time
import paho.mqtt.client as mqtt
import Adafruit_DHT
import RPi.GPIO as GPIO
from gpiozero import LED
from tkinter import *
from threading import Thread

PIR_status = False
humidity = 50
temperature = 25 
distance = 0


PIR_pin = 26 #GPIO number
led_pin = 20
DHT11_pin = 19
sr04_trig_pin = 23
sr04_echo_pin = 24

led = LED(led_pin)

topic_temp = "base/state/temperature"
topic_humidity = "base/state/humidity"
topic_seed_hight = "base/state/seed_hight"
topic_movement = "base/state/movement"
topic_led1 = "base/relay/led1"
topic_ask_temp = "base/relay/ask_temp"

#############GUI#############
window = Tk()
window.title("Показания датчиков")
window.geometry("300x200")

def update_label(label, text):
 label.config(state="normal")
 label.delete(0, END)
 label.insert(0, str(text))
 label.config(state="readonly")

#датчик1
s1_label = Label(window, text="Температура ")
s1_label.grid(column=0, row=0)
s1_info = Entry(window, state="readonly")
s1_info.grid(column=1, row=0)  
update_label(s1_info, "0")

#датчик2
s2_label = Label(window, text="Влажность ")
s2_label.grid(column=0, row=1)
s2_info = Entry(window, state="readonly")
s2_info.grid(column=1, row=1)  
update_label(s2_info, "0")

#датчик3
s3_label = Label(window, text="Движение ")
s3_label.grid(column=0, row=2)
s3_info = Entry(window, state="readonly")
s3_info.grid(column=1, row=2)  
update_label(s3_info, "Нет")

#датчик4
s4_label = Label(window, text="Расстояние ")
s4_label.grid(column=0, row=3)
s4_info = Entry(window, state="readonly")
s4_info.grid(column=1, row=3)  
update_label(s4_info, "0")

#датчик5
#s5_label = Label(window, text="Температура ")
#s5_label.grid(column=0, row=4)
#s5_info = Entry(window, state="readonly")
#s5_info.grid(column=1, row=4)  
#update_label(s5_info, "123")

def loop_main():
 while True:
  update_label(s1_info, temperature)
  update_label(s2_info, humidity)
  if PIR_status:
   update_label(s3_info, "Да")
  else:
   update_label(s3_info, "Нет")
  update_label(s4_info, distance)
  window.update()
  sleep(0.2)

#############Network#############
client_id="mqtt-dugin_arseniy-rpizero1"
mqtt_brocker = "sandbox.rightech.io"

def on_connect(client, userdata, flags, rc):
 print("Connected")
 client.subscribe(topic_led1)
 client.subscribe(topic_ask_temp)

def on_message(client, userdata, message):
 if str(message.topic) == topic_led1 and str(message.payload.decode("utf-8")) == "1":
  led.on()
  client.publish(topic_led1, payload="ok")
 if str(message.topic) == topic_led1 and str(message.payload.decode("utf-8")) == "0":
  led.off()
  client.publish(topic_led1, payload="ok")
 if str(message.topic) == topic_ask_temp and str(message.payload.decode("utf-8")) == "1":
  client.publish(topic_temp, payload=temperature)
  client.publish(topic_humidity, payload=humidity)
  print("Temp sent")

def mqqt_loop(client):
 sleep(5)
 last_PIR_status = False
 last_distance = 0
 #client.publish(topic_movement, payload=PIR_status)
 #client.publish(topic_seed_hight, payload=distance)
 while True:
  if PIR_status != last_PIR_status:
   client.publish(topic_movement, payload=PIR_status)
   last_PIR_status = PIR_status
  if distance != last_distance:
   client.publish(topic_seed_hight, payload=distance)
   last_distance = distance
  sleep(0.1)


#инициализация
mqttc = mqtt.Client(client_id=client_id)
mqttc.connect(host=mqtt_brocker, port=1883, keepalive=60)
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.loop_start() #запуск обработки on_message

#переподключение
#mqttc.reconnect()

#отключение
#mqttc.disconnect()


#############Sensors#############

def get_distance(trigger_pin, echo_pin): 
 global distance
 GPIO.setup(trigger_pin, GPIO.OUT)
 GPIO.setup(echo_pin, GPIO.IN)
 while True:
  GPIO.output(trigger_pin, True)
  sleep(0.00001)
  GPIO.output(trigger_pin, False)
  timeout = time()
  error_state = False
  while GPIO.input(echo_pin) == 0:
   start_time = time()
   if start_time - timeout > 0.025:
    error_state = True
    break
  while GPIO.input(echo_pin) == 1:
   stop_time = time()
  if not error_state and (stop_time - start_time) > 0 and (stop_time - start_time) < 400:
   distance = round(((stop_time - start_time) * 34300) / 2, 1)
  sleep(0.5)

def get_PIR_status(pin):
 global PIR_status
 GPIO.setup(pin, GPIO.IN)
 while True:
  if GPIO.input(pin) == 0:
   PIR_status = False
  elif GPIO.input(pin) == 1:
   PIR_status = True
  sleep(0.5)
 
def get_temp_hum(pin):
 global humidity, temperature
 temp_sensor = Adafruit_DHT.DHT11
 while True:
  humidity, temperature = Adafruit_DHT.read_retry(sensor=temp_sensor, pin = pin, delay_seconds=1)
  print(humidity, " ", temperature)
  sleep(0.5)
  
#############Start#############

thread_DHT = Thread(target=get_temp_hum, args=(DHT11_pin,))
thread_PIR = Thread(target=get_PIR_status, args=(PIR_pin,))
thread_SR04 = Thread(target=get_distance, args=(sr04_trig_pin, sr04_echo_pin,))
thread_MQTT = Thread(target=mqqt_loop, args=(mqttc,))

thread_PIR.start()
thread_DHT.start()
thread_SR04.start()
thread_MQTT.start()
window.after(3000, loop_main())
window.mainloop()



thread_DHT.join()
thread_PIR.join()
thread_SR04.join()