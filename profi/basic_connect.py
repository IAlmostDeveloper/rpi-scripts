import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
from time import sleep
client_id="mqtt-dugin_arseniy-oonr5q"
mqtt_brocker = "sandbox.rightech.io"
#инициализация
mqttc = mqtt.Client(client_id=client_id)
mqttc.connect(host=mqtt_brocker, port=1883, keepalive=60)


#подписка
topic_temp = "base/state/temperature"
topic_humidity = "base/state/humidity"
topic_seed_hight = "base/state/seed_hight"
topic_movement = "base/state/movement"
topic_led1 = "base/relay/led1"
topic_ask_temp = "base/relay/ask_temp"

def on_connect(client, userdata, flags, rc):
 print("Connected")
 client.subscribe(topic_led1)
 client.subscribe(topic_ask_temp)

def on_message(client, userdata, message):
 if message.topic == topic_ask_temp:
  client.publish(topic_temp, payload=25)
  client.publish(topic_humidity, payload=50)
  print("Temp sent")

mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.loop_start()

while True:
 sleep(1)