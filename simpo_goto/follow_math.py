#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
from pymavlink import mavutil
import utils
import getch

vehicle = connect('127.0.0.1:14570', wait_ready=True, baud=115200) #與飛機連線
first_vehicle = connect('127.0.0.1:14551', wait_ready=True, baud=115200) #與飛機連線

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

def target(dis):
    lat = first_vehicle.location.global_relative_frame.lat #無人機緯度座標
    lon = first_vehicle.location.global_relative_frame.lon #無人機經度座標
    alt = first_vehicle.location.global_relative_frame.alt
    print(vehicle.attitude.pitch) #顯示無人機pitch角度
    droneheading = math.radians(90-first_vehicle.heading-135) #飛機頭向
    height = 100 #飛機高度
    distance = dis
    print("離目標物距離:",distance)
    earth_radius=6378137.0 #地球半徑
    lat1 = distance*math.sin(droneheading)
    lon1 = distance*math.cos(droneheading)
    print(lat1,lon1)
    dlat = lat1/earth_radius
    dlon = lon1/(earth_radius*math.cos(math.pi*lat/180))

    newlat = lat + (dlat * 180/math.pi)
    newlon = lon + (dlon * 180/math.pi)
    print("new",newlat,newlon)

    distancelat = newlat - lat
    distancelon = newlon - lon
    get_distance_metres = math.sqrt((distancelat**2)+(distancelon**2))* 1.113195e5
    print("座標離目標物距離:",get_distance_metres)

    return LocationGlobalRelative(newlat, newlon, alt)

def leader():
    leaderlat = first_vehicle.location.global_relative_frame.lat #無人機緯度座標
    leaderlon = first_vehicle.location.global_relative_frame.lon #無人機經度座標
    leaderalt = first_vehicle.location.global_relative_frame.alt
    followlat = vehicle.location.global_relative_frame.lat #無人機緯度座標
    followlon = vehicle.location.global_relative_frame.lon #無人機經度座標
    followalt = vehicle.location.global_relative_frame.alt
    droneheading = math.radians(90-first_vehicle.heading) #飛機頭向
    lx = -(2-1)*math.cos(20)-(2-1)*math.sin(20)
    print(lx)
    return lx

leader()
# if vehicle.armed != True:
#     utils.arm_and_takeoff(first_vehicle,vehicle,20) #起飛高度
#     print("takeoff")
# else:
#     vehicle.mode = VehicleMode("GUIDED")
#     print("change mode")

# # print("Set default/target airspeed to 8")
# # vehicle.airspeed = 8

# while True:
#     start = time.time()
#     global remainingDistance, targetDistance
#     #point2 = goto(-20,0,alt-5) #往南飛20公尺
#     a = target(10)
#     vehicle.simple_goto(a)
#     time.sleep(0.5)
#     end = time.time()
#     print("執行時間:"+ str(end-start))

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
first_vehicle.close()
