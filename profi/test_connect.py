from paho.mqtt import client

client_id="mqtt-dugin_arseniy-rpizero"
mqtt_brocker = "sandbox.rightech.io"
def on_connect(client, userdata, rc):  
    print("Connected with result code: %s" % rc)
    

def on_message(client, userdata, msg):  
 print("%s: %s" % (msg.topic, msg.payload))


subscriber = client.Client(client_id=client_id)
subscriber.on_connect = on_connect
subscriber.on_message = on_message
subscriber.connect("dev.rightech.io")
subscriber.subscribe("base/relay/ask_temp")
subscriber.loop_forever()
