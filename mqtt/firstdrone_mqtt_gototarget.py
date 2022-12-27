from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
from pymavlink import mavutil
import utils
import getch
import paho.mqtt.client as mqtt
import json

first_vehicle = connect('127.0.0.1:14561', wait_ready=True, baud=115200) #與飛機連線

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # 將訂閱主題寫在on_connet中
    # 如果我們失去連線或重新連線時 
    # 地端程式將會重新訂閱
    client.subscribe("target/position")

def on_message(client, userdata, msg):
    # 轉換編碼utf-8才看得懂中文
    print(msg.topic+" "+ msg.payload.decode('utf-8'))
    target1 = position(msg.payload.decode('utf-8'))
    print(target1[0])
    a = LocationGlobalRelative(target1[0], target1[1], 10)
    first_vehicle.simple_goto(a)
    client.loop_stop()

def position(data):
    data = json.loads(data)
    Lat = data["Latitude"]
    Lon = data["Longtitude"]
    return Lat,Lon

client = mqtt.Client()

client.on_connect = on_connect

client.on_message = on_message

client.username_pw_set("bighead1","nfuaesil1")

client.connect("192.168.0.117", 1883, 60)

client.loop_forever()