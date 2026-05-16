"""
MQTT Subscriber Test — run this on your PC to check if the gateway is publishing.

Install first:  pip install paho-mqtt
Run:            python mqtt_test_sub.py
"""
import paho.mqtt.client as mqtt
import time

BROKER = "broker.emqx.io"
PORT = 1883
TOPIC = "delta/gw/data"

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[OK] Connected to {BROKER}:{PORT}")
        client.subscribe(TOPIC)
        print(f"[OK] Subscribed to topic: {TOPIC}")
        print(f"[..] Waiting for messages... (Ctrl+C to quit)\n")
    else:
        print(f"[ERR] Connection failed with code {rc}")

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode('utf-8', errors='replace')
    except:
        payload = msg.payload.hex()
    print(f"[MSG] Topic: {msg.topic}")
    print(f"      Payload ({len(msg.payload)} bytes): {payload}")
    print()

client = mqtt.Client(client_id="test-sub-pc-001")
client.on_connect = on_connect
client.on_message = on_message

print(f"Connecting to {BROKER}:{PORT}...")
client.connect(BROKER, PORT, 60)

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nDisconnected.")
    client.disconnect()
