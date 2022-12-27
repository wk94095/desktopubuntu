#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
import json
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Attitude
from utils.gimbalmath import target
from utils.drone import arm_and_takeoff
import paho.mqtt.client as mqtt
start = time.time()
vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線

# 連線設定
# 初始化地端程式
client = mqtt.Client()

# 設定登入帳號密碼
client.username_pw_set("bighead","nfuaesil")

# 設定連線資訊(IP, Port, 連線時間)
client.connect("192.168.0.117", 1883, 60)

#arm_and_takeoff(vehicle, 5)

#for _ in range(3):
a = target(vehicle,10)
print(a.lat)
payload = {'Latitude' : a.lat, 'Longtitude' : a.lon}
print(json.dumps(payload))
client.publish("target/position", json.dumps(payload))
    #time.sleep(8)
#vehicle.simple_goto(a)
# print(vehicle.location.global_relative_frame)
# print(vehicle.heading)



vehicle.close()
end = time.time()

print("執行時間: %f 秒" %(end-start))