# test_base_station.py
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to broker. Listening for rover data...")
    client.subscribe("rover/#") # The '#' wildcard subscribes to ALL rover topics

def on_message(client, userdata, msg):
    print(f"[{msg.topic}]: {msg.payload.decode()}")

client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

client.connect("localhost", 1883, 60)
client.loop_forever()