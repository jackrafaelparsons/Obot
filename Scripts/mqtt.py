import paho.mqtt.client as mqtt
import threading
import queue
import time


# Create a thread-safe queue for passing messages between threads
message_queue = queue.Queue()
 
# Define the callback function for when a message is received
def on_message(client, userdata, message):
    global last_fuel, last_plate, fuel, plate
    if message.topic == "Obot/confirm":
        confirm_angle = int(message.payload)
        return confirm_angle




# Create an MQTT client and connect to the broker
client = mqtt.Client()
 
# Set the callback function for when a message is received
client.on_message = on_message
 
# Set the username and password
client.username_pw_set("student",password="HousekeepingGlintsStreetwise")
client.connect("fesv-mqtt.bath.ac.uk",31415,60)
print("Connected to MQTT broker")
 
# Subscribe to the topic we're interested in
client.subscribe("Obot/confirm")
 
# Write a message to the topic we're interested in
# client.publish("my/topic", "1")
 
direction = 0
client.publish("Obot/movement", str(direction))





# Start the MQTT client loop
client.loop_forever()