from matplotlib.pyplot import plot
from ca_utils import *
import paho.mqtt.client as mqttClient
import time

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        global Connected                #Use global variable
        Connected = True                #Signal connection 
    else:
        print("Connection failed")

def on_message(client, userdata, message):
    list = message.payload.decode("utf-8").split("|")
    lat, lon = float(list[1]), float(list[3])
    if message.topic == "/54321/1/attrs":
        object_1.update_position(lat, lon)
    else:
        object_2.update_position(lat,lon)
    
    print(lat, lon)
    print(object_1.heading_deg)
    print(object_1.xy)
    

object_1 = moving_object(origin[0],origin[1],1,type="Tractor")
object_2 = moving_object(origin[0],origin[1],2,type="Spraying drone")

figure, ax = plt.subplots(figsize=(20,20))
plt.axis('equal')
plt.ion()
plt.xlim([-100, 400])
plt.ylim([-100, 400])
ax.set_autoscale_on(False)





Connected = False   #global variable for the state of the MQTT connection
client = mqttClient.Client("CA")
client.on_connect= on_connect       #attach function to callback (connection)
client.on_message= on_message       #attach function to callback (message received)
client.connect("flexigrobots.collab-cloud.eu", port = 1883)
client.loop_start()        #start the loop 
while Connected != True:    #Wait for connection
    time.sleep(0.1)
client.subscribe("/54321/2/attrs")
client.subscribe("/54321/1/attrs")

while True:
    time.sleep(0.5)
    