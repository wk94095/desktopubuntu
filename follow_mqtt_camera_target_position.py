#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
import json
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Attitude
from pymavlink import mavutil
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import threading
start = time.time()
flag = 0
vehicle = connect('127.0.0.1:14561', wait_ready=True, baud=115200) #與飛機連線

def arm_and_takeoff(aTargetAltitude): #定義起飛程序
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break

        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2): #定義目標位置與目標位置計算出距離
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def aux(ACTUATOR,pwm): #設定aux通道
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        ACTUATOR,pwm,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def aa():
    client = mqtt.Client()

    # 設定登入帳號密碼
    client.username_pw_set("bighead","nfuaesil")

    # 設定連線資訊(IP, Port, 連線時間)
    client.connect("192.168.0.117", 1883, 60) #IP需換成serverIP
    # 準備要傳送的訊息
    while True:
        msg = {'lat' : vehicle.location.global_relative_frame.lat ,
                'lon' : vehicle.location.global_relative_frame.lon,
                'alt': vehicle.location.global_relative_frame.alt,
                'V' : "{:.2f}".format(vehicle.battery.voltage),
                'heading' : vehicle.heading,
                'flightmode': vehicle.mode.name,
                'AirSpeed':"{:.2f}".format(vehicle.airspeed),
                'armed' : vehicle.armed, 
                'name' : 'leader'}
        client.publish("drone/leader", json.dumps(msg))
        if vehicle.mode.name == "RTL" and vehicle.armed == False:
            payload = {"lat" : None}
            client.publish("drone/leader", json.dumps(payload))
            vehicle.mode = VehicleMode("STABILIZE")
            break
        time.sleep(0.3)

def bb():
    # 當地端程式連線伺服器得到回應時，要做的動作
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # 將訂閱主題寫在on_connet中
        # 如果我們失去連線或重新連線時 
        # 地端程式將會重新訂閱
        client.subscribe("target/position")
        

    def on_connect1(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # 將訂閱主題寫在on_connet中
        # 如果我們失去連線或重新連線時 
        # 地端程式將會重新訂閱
        client.subscribe("drone/throw")

    # 當接收到從伺服器發送的訊息時要進行的動作
    def on_message(client, userdata, msg):
        global target1, flag
        # 轉換編碼utf-8才看得懂中文
        #time.sleep(0.5)
        print(msg.topic+" "+ msg.payload.decode('utf-8'))
        target1 = position(msg.payload.decode('utf-8'))
        print(target1)
        payload = {"goit" : None}
        client.publish("drone/goit1",json.dumps(payload))
        while True:
            if vehicle.mode.name != "RTL":
                if vehicle.armed == True:
                    a = LocationGlobalRelative(target1[0], target1[1], 10)
                    vehicle.simple_goto(a)
                    distancepoint = get_distance_metres(vehicle.location.global_relative_frame,a)
                    print(distancepoint)
                    if flag == 0:
                        if distancepoint <= 10:
                            flag = 1
                    if distancepoint*0.95 <= 1 and vehicle.airspeed < 1:
                        # aux(12,875)
                        # print('on')
                        # time.sleep(1)
                        # aux(12,1900)
                        # print('off')
                        print ("Reached target")
                        payload = {"goit" : "drone1goit"}
                        client.publish("drone/goit1", json.dumps(payload))
                        break
                    time.sleep(1)
                else:
                    arm_and_takeoff(5)
                    a = LocationGlobalRelative(target1[0], target1[1], 10)
                    vehicle.simple_goto(a)
                    distancepoint = get_distance_metres(vehicle.location.global_relative_frame,a)
                    print(distancepoint)
            else:
                break
    
    def on_message1(client, userdata, msg):
        global target1, flag
        # 轉換編碼utf-8才看得懂中文
        #time.sleep(0.5)
        print(msg.topic+" "+ msg.payload.decode('utf-8'))
        data = json.loads(msg.payload.decode('utf-8'))
        if msg.topic == 'drone/throw':
            if data['throw'] == "on":
                print('on')
                aux(12,1900)
                print("relayon")
                time.sleep(1)
                aux(12,1000)
                print('relayoff')
                
            else:
                print('off')

    def position(data):
        data = json.loads(data)
        Lat = data.get("lat")
        Lon = data.get("lon")
        return Lat,Lon

    # 連線設定
    # 初始化地端程式
    client = mqtt.Client()
    #client = mqtt.Client()

    # 設定連線的動作
    client.on_connect = on_connect
    client.on_connect = on_connect1

    # 設定接收訊息的動作
    client.on_message = on_message
    client.on_message = on_message1

    # 設定登入帳號密碼
    client.username_pw_set("bighead1","nfuaesil1")
    client.username_pw_set("bighead1","nfuaesil1")

    # 設定連線資訊(IP, Port, 連線時間)
    #try:
    client.connect("192.168.0.117", 1883, 60)
    client.connect("192.168.0.117", 1883, 60)
    # except:
    #     client.connect("192.168.0.117", 1883, 60)
    end = time.time()
    print("執行時間: %f 秒" %(end-start))
    #client.loop_forever()
    #print(flag)
    # 開始連線，執行設定的動作和處理重新連線問題
    # 也可以手動使用其他loop函式來進行連接
    while True:
        client.loop_start()
        #client1.loop_start()
        if flag == 1:
            payload = {'mode' : "triangle"}
            print("flag")
            client.publish("drone/mode",json.dumps(payload))
            break
            #flag = 0
        #time.sleep(1)
            
        
a = threading.Thread(target=aa)  # 建立新的執行緒
b = threading.Thread(target=bb)  # 建立新的執行緒

a.start()  # 啟用執行緒
b.start()  # 啟用執行緒

#vehicle.close()