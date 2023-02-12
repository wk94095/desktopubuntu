#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
from pymavlink import mavutil
import utils
import getch
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import json
import threading

vehicle = connect('127.0.0.1:14590', wait_ready=True, baud=115200) #與飛機連線
first_vehicle = connect('127.0.0.1:14563', wait_ready=True, baud=115200) #與飛機連線
home = 0
mqtthost = "192.168.0.117"
mqttport = 1883
mqtttopic = "drone/follow3"
def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)
    time.sleep(1)

def goto(dNorth, dEast, dAlt, gotoFunction=vehicle.simple_goto): #dNorth 數值正往北飛負往南 dEast數值正往東飛負往西
    global remainingDistance, targetDistance

    currentLocation=first_vehicle.location.global_relative_frame #當前位置
    targetLocation=utils.get_location_metres(currentLocation, dNorth, dEast, dAlt)
    targetDistance=utils.get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    remainingDistance=utils.get_distance_metres(vehicle.location.global_frame, targetLocation)

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
    return pwm

def target(dis, dhead, dalt):
    lat = first_vehicle.location.global_relative_frame.lat #無人機緯度座標
    lon = first_vehicle.location.global_relative_frame.lon #無人機經度座標
    alt = dalt
    #print(vehicle.attitude.pitch) #顯示無人機pitch角度
    droneheading = math.radians(90-first_vehicle.heading-dhead) #飛機頭向
    distance = dis
    #print("離目標物距離:",distance)
    earth_radius=6378137.0 #地球半徑
    lat1 = distance*math.sin(droneheading)
    lon1 = distance*math.cos(droneheading)
    #print(lat1,lon1)
    dlat = lat1/earth_radius
    dlon = lon1/(earth_radius*math.cos(math.pi*lat/180))

    newlat = lat + (dlat * 180/math.pi)
    newlon = lon + (dlon * 180/math.pi)
    #print("new",newlat,newlon)

    # distancelat = newlat - lat
    # distancelon = newlon - lon
    # get_distance_metres = math.sqrt((distancelat**2)+(distancelon**2))* 1.113195e5
    #print("座標離目標物距離:",get_distance_metres)

    return LocationGlobalRelative(newlat, newlon, alt)

def aa():
    # 準備要傳送的訊息
    while True:
        msg = { 'lat' : vehicle.location.global_relative_frame.lat ,
                'lon' : vehicle.location.global_relative_frame.lon,
                'alt': vehicle.location.global_relative_frame.alt,
                'V' : "{:.2f}".format(vehicle.battery.voltage),
                'heading' : vehicle.heading,
                'flightmode': vehicle.mode.name,
                'AirSpeed':"{:.2f}".format(vehicle.airspeed),
                'armed' : vehicle.armed, 
                'name' : 'follow3'}
        messages = [
        {'topic':mqtttopic, 'payload': json.dumps(msg)}
        ]
        # 發布多則 MQTT 訊息
        publish.multiple(
        messages,
        hostname = mqtthost,
        port = mqttport,
        auth={'username':'bighead','password':'nfuaesil'})
        time.sleep(0.3)
        if vehicle.mode.name == "RTL" and vehicle.armed == False:
            msg = { 'lat' : None}
            messages = [
            {'topic':mqtttopic, 'payload': json.dumps(msg)}
            ]
            # 發布多則 MQTT 訊息
            publish.multiple(
            messages,
            hostname = mqtthost,
            port = mqttport,
            auth={'username':'bighead','password':'nfuaesil'})
            print("88")
            break
    vehicle.mode = VehicleMode("STABILIZE")

def bb():
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # 將訂閱主題寫在on_connet中
        # 如果我們失去連線或重新連線時 
        # 地端程式將會重新訂閱
        client.subscribe("drone/mode")
        client.subscribe("drone/throw")

    # 當接收到從伺服器發送的訊息時要進行的動作
    def on_message(client, userdata, msg):
        # 轉換編碼utf-8才看得懂中文
        print(msg.topic+" "+ msg.payload.decode('utf-8'))
        data = json.loads(msg.payload.decode('utf-8'))
        mode = data.get("mode")
        throw = data.get("throw")
        firstalt = first_vehicle.location.global_relative_frame.alt
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
        if mode == 'line':
            start = time.time()
            payload = {"goit" : None}
            client.publish("drone/goit4", json.dumps(payload))
            point = target(6, -180,firstalt-8)
            vehicle.simple_goto(point,3)
            time.sleep(0.1)
            distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
            if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
                #vehicle.simple_goto(point1)
                print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
                if first_vehicle.mode == "RTL":
                    print("Returning to Launch")
                    vehicle.mode = VehicleMode("LAND")
                    #break
            elif first_vehicle.mode == "RTL":
                print("Returning to Launch")
                vehicle.mode = VehicleMode("LAND")
                #break
            elif distancetopoint*0.95<=1:
                print ("Reached target")

            else:
                print("Change Mode Guided")
                vehicle.mode = VehicleMode("GUIDED")
            end = time.time()
            #print("執行時間:"+ str(end-start))
        elif mode == 'triangle':
            start = time.time()
            payload = {"goit" : None}
            client.publish("drone/goit4", json.dumps(payload))
            point = target(4, -135,firstalt-8)
            vehicle.simple_goto(point,3)
            time.sleep(0.1)
            distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
            if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
                print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
                if first_vehicle.mode == "RTL":
                    print("Returning to Launch")
                    vehicle.mode = VehicleMode("LAND")
                    #break
            elif first_vehicle.mode == "RTL":
                print("Returning to Launch")
                vehicle.mode = VehicleMode("LAND")
                #break
            elif distancetopoint*0.95<=1:
                print ("Reached target")
                payload = {"goit" : "drone4goit"}
                client.publish("drone/goit4", json.dumps(payload))
                if throw == 'on':
                    print("on")
            else:
                print("Change Mode Guided")
                vehicle.mode = VehicleMode("GUIDED")
        elif mode == 'Inverted triangle':
            start = time.time()
            payload = {"goit" : None}
            client.publish("drone/goit4", json.dumps(payload))
            point = target(4, -45,firstalt-8)
            vehicle.simple_goto(point,3)
            time.sleep(0.1)
            distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
            if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
                #vehicle.simple_goto(point1)
                print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
                if first_vehicle.mode == "RTL":
                    print("Returning to Launch")
                    vehicle.mode = VehicleMode("LAND")
                    #break
            elif first_vehicle.mode == "RTL":
                print("Returning to Launch")
                vehicle.mode = VehicleMode("LAND")
                #break
            elif distancetopoint*0.95<=1:
                print ("Reached target")

            else:
                print("Change Mode Guided")
                vehicle.mode = VehicleMode("GUIDED")
        elif mode == 'side_line':
            start = time.time()
            payload = {"goit" : None}
            client.publish("drone/goit4", json.dumps(payload))
            point = target(6, -90,firstalt-8)
            vehicle.simple_goto(point,3)
            time.sleep(0.1)
            distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
            if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
                #vehicle.simple_goto(point1)
                print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
                if first_vehicle.mode == "RTL":
                    print("Returning to Launch")
                    vehicle.mode = VehicleMode("LAND")
                    #break
            elif first_vehicle.mode == "RTL":
                print("Returning to Launch")
                vehicle.mode = VehicleMode("LAND")
                #break
            elif distancetopoint*0.95<=1:
                print ("Reached target")
            else:
                print("Change Mode Guided")
                vehicle.mode = VehicleMode("GUIDED")
            end = time.time()
            #print("執行時間:"+ str(end-start))

    if vehicle.armed != True:
        utils.arm_and_takeoff(first_vehicle,vehicle,10) #起飛高度
        print("takeoff")

    else:
        vehicle.mode = VehicleMode("GUIDED")
        print("change mode")
    # 連線設定
    # 初始化地端程式
    client = mqtt.Client()

    # 設定連線的動作
    client.on_connect = on_connect

    # 設定接收訊息的動作
    client.on_message = on_message

    # 設定登入帳號密碼
    client.username_pw_set("bighead","nfuaesil")

    # 設定連線資訊(IP, Port, 連線時間)
    client.connect("192.168.0.117", 1883, 60)
    while vehicle.mode.name != "RTL":
        client.loop_start()
        #print("close")
    client.loop_stop()
    client.disconnect()

a = threading.Thread(target=aa)  # 建立新的執行緒
b = threading.Thread(target=bb)  # 建立新的執行

a.start()  # 啟用執行緒
b.start()  # 啟用執行緒
# Close vehicle object before exiting script
print("Close vehicle object")
#vehicle.close()
#first_vehicle.close()
