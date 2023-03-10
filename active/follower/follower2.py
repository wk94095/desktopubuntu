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
mqtthost = ("192.168.0.101")
mqttport = 1883
#vehicle = connect('192.168.0.115:14550', wait_ready=True, baud=115200) #與飛機連線
vehicle = connect('127.0.0.1:14571', wait_ready=True, baud=115200) #與飛機連線

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

def relay(Instance,setting):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY, #command
        0, #confirmation
        Instance,setting,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def message():
    client = mqtt.Client()

    # 設定登入帳號密碼
    client.username_pw_set("bighead","nfuaesil")

    # 設定連線資訊(IP, Port, 連線時間)
    client.connect(mqtthost, mqttport, 60) #IP需換成serverIP
    # 準備要傳送的訊息

    while True:
        msg = { 'lat' : vehicle.location.global_relative_frame.lat ,
                'lon' : vehicle.location.global_relative_frame.lon,
                'alt': vehicle.location.global_relative_frame.alt,
                'V' : "{:.2f}".format(vehicle.battery.voltage),
                'heading' : vehicle.heading,
                'flightmode': vehicle.mode.name,
                'AirSpeed': "{:.2f}".format(vehicle.airspeed),
                'armed' : vehicle.armed,
                'name' : 'leader2'}
        messages = [
        {'topic':"drone/leader2", 'payload': json.dumps(msg)},
        {'topic':"switch", 'payload': True}
        ]
        # 發布多則 MQTT 訊息
        publish.multiple(
        messages,
        hostname = mqtthost,
        port = mqttport,
        auth={'username':'bighead1','password':'nfuaesil1'})
        time.sleep(0.3)
        if vehicle.mode.name == "RTL" and vehicle.armed == False:
            msg = { 'lat' : None}
            messages = [
            {'topic': "drone/leader2", 'payload': json.dumps(msg)},
            {'topic':"switch", 'payload': False}
            ]
            # 發布多則 MQTT 訊息
            publish.multiple(
            messages,
            hostname = mqtthost,
            port = mqttport,
            auth={'username':'bighead1','password':'nfuaesil1'})
            print("88")
            break
    vehicle.mode = VehicleMode("LOITER")

def control():
    # 當地端程式連線伺服器得到回應時，要做的動作
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # 將訂閱主題寫在on_connet中
        # 如果我們失去連線或重新連線時 
        # 地端程式將會重新訂閱
        client.subscribe("drone1/goto")
        client.subscribe("target/position")

    def on_connect1(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # 將訂閱主題寫在on_connet中
        # 如果我們失去連線或重新連線時 
        # 地端程式將會重新訂閱
        client1.subscribe("drone/throw")

    # 當接收到從伺服器發送的訊息時要進行的動作
    def on_message(client, userdata, msg):
        global target1, flag
        # 轉換編碼utf-8才看得懂中文
        #time.sleep(0.5)
        print(msg.topic+" "+ msg.payload.decode('utf-8'))
        target1 = position(msg.payload.decode('utf-8'))
        data = json.loads(msg.payload.decode('utf-8'))
        print(data.get('drone2'))
        print(target1)
        while data.get('drone2') == True:
            if vehicle.mode.name != "RTL":
                if vehicle.armed == True:
                    a = LocationGlobalRelative(target1[0], target1[1], 20)
                    vehicle.simple_goto(a)
                    distancepoint = get_distance_metres(vehicle.location.global_relative_frame,a)
                    print(distancepoint)
                    if distancepoint*0.95 <= 1 and vehicle.airspeed < 1:
                        print ("Reached target")
                        payload = {"goit" : "drone1goit"}
                        client.publish("drone/leadergoit", json.dumps(payload))
                        break
                    time.sleep(1)
                else:
                    arm_and_takeoff(15)
                    a = LocationGlobalRelative(target1[0], target1[1], 20)
                    vehicle.simple_goto(a)
                    distancepoint = get_distance_metres(vehicle.location.global_relative_frame,a)
                    print(distancepoint)
            else:
                payload = {"goit" : None}
                client.publish("drone/leadergoit", json.dumps(payload))
                break
    
    def on_message1(client, userdata, msg):
        global target1, flag
        # 轉換編碼utf-8才看得懂中文
        #time.sleep(0.5)
        print(msg.topic+" "+ msg.payload.decode('utf-8'))
        data = json.loads(msg.payload.decode('utf-8'))
        if msg.topic == 'drone/throw':
            if data['throw'] == "on":
                relay(0,1)
                print("fire")
                time.sleep(1)
                relay(0,0)
                aux(9,1900)
                print("relayon")
                time.sleep(1)
                aux(9,1000)
                print('relayoff')
                payload = {"goit" : None}
                client.publish("drone/leadergoit", json.dumps(payload))
                time.sleep(1)
                vehicle.mode = VehicleMode("RTL")
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
    client1 = mqtt.Client()
    # 設定連線的動作
    client.on_connect = on_connect
    client1.on_connect = on_connect1
    # 設定接收訊息的動作
    client.on_message = on_message
    client1.on_message = on_message1
    # 設定登入帳號密碼
    client.username_pw_set("bighead1","nfuaesil1")
    client1.username_pw_set("bighead1","nfuaesil1")
    # 設定連線資訊(IP, Port, 連線時間)
    #try:
    client.connect(mqtthost, mqttport, 60)
    client1.connect(mqtthost, mqttport, 60)
    # except:
    #     client.connect("192.168.0.117", 1883, 60)
    end = time.time()
    print("執行時間: %f 秒" %(end-start))
    #client.loop_forever()
    #print(flag)
    # 開始連線，執行設定的動作和處理重新連線問題
    # 也可以手動使用其他loop函式來進行連接
    while vehicle.mode.name != "RTL":
        client.loop_start()
        client1.loop_start()
        
    print("close")
    client.loop_stop()
    client1.loop_stop()
    
        
        
send = threading.Thread(target=message)  # 建立新的執行緒
main = threading.Thread(target=control)  # 建立新的執行緒

send.start()  # 啟用執行緒
main.start()  # 啟用執行緒

#vehicle.close()
