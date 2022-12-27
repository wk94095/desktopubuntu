#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
from pymavlink import mavutil
import utils
import getch

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
first_vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=115200) #與飛機連線

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

if vehicle.armed != True:
    utils.arm_and_takeoff(first_vehicle,vehicle,20) #起飛高度
    print("takeoff")
else:
    vehicle.mode = VehicleMode("GUIDED")
    print("change mode")

# print("Set default/target airspeed to 8")
# vehicle.airspeed = 8

while True:
    global remainingDistance, targetDistance
    lat = first_vehicle.location.global_relative_frame.lat #讀取掌機緯度座標
    lon = first_vehicle.location.global_relative_frame.lon #讀取掌機經度座標
    alt = first_vehicle.location.global_relative_frame.alt #讀取掌機高度
    point1 = LocationGlobalRelative(lat, lon, 20)
    point2 = goto(-20,0,10) #往南飛20公尺
    time.sleep(0.5)
    if remainingDistance >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
        #vehicle.simple_goto(point1)
        print("Distance to target:"+"{:.2f}".format(remainingDistance)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
        if first_vehicle.mode == "RTL":
            print("Returning to Launch")
            vehicle.mode = VehicleMode("RTL")
            break
    #elif ord(getch.getch()) in [81,113]:
    elif first_vehicle.mode == "RTL":
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        break
    elif remainingDistance<=targetDistance*0.1:
        print ("Reached target")
        
    else:
        print("Change Mode Guided")
        vehicle.mode = VehicleMode("GUIDED")  

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
first_vehicle.close()
