import matplotlib.pyplot as plt
from ca_utils import *
import paho.mqtt.client as mqttClient
import csv

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
    h = float(list[7])

    with open ("log.csv", "a+") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([message.topic]+message.payload.decode("utf-8").split("|"))

    if message.topic == "/54321/1/attrs":
        object_1.update_position(lat, lon,h)
    else:
        object_2.update_position(lat,lon,h)

    plot_object_lines(object_1,object_2,r1, r1_ss,r2, r2_ss,p1,p2)
    res_str = "Collision time interval: "+str(moving_separating_axis_theorem(object_1, object_2))+"\n"\
        + "Vehicle collision state: "+ str(separating_axis_theorem(object_1,object_2,super_safe=False))+"\n"\
        + "Safe areas collision state: "+str(separating_axis_theorem(object_1, object_2))
    ax.set_title(res_str)

object_1 = moving_object(origin[0],origin[1],1,type="Tractor")
object_2 = moving_object(origin[0],origin[1],2,type="Spraying drone")

figure, ax = plt.subplots(figsize=(10,10))
plt.axis('equal')
plt.axis([-1000, 1000, -1000, 1000])
ax.set_autoscale_on(False)

zero_polygon = ([0,0,0,0],[0,0,0,0])

plt.ion()
p1 = plt.plot(0,0,".",color = "red")[0]
p2 = plt.plot(0,0,".",color = "blue")[0]
r1 = plt.plot(zero_polygon,c = "red")[0]
r2 = plt.plot(zero_polygon,c = "blue")[0]
r1_ss = plt.plot(zero_polygon,"--",c = "red")[0]
r2_ss = plt.plot(zero_polygon,"--", c = "blue")[0]

Connected = False   #global variable for the state of the MQTT connection
client = mqttClient.Client("CA")
client.on_connect= on_connect       #attach function to callback (connection)
client.on_message= on_message       #attach function to callback (message received)
client.connect("flexigrobots.collab-cloud.eu", port = 1883)
client.loop_start()        #start the loop 
while Connected != True:    #Wait for connection
    plt.pause(0.1)
client.subscribe("/54321/2/attrs")
client.subscribe("/54321/1/attrs")

while True:
    plt.pause(100)
    