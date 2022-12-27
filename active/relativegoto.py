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
home = 0
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

if vehicle.armed != True:
    utils.arm_and_takeoff(first_vehicle,vehicle,10) #起飛高度
    print("takeoff")
    
    
else:
    vehicle.mode = VehicleMode("GUIDED")
    print("change mode")

# print("Set default/target airspeed to 8")
# vehicle.airspeed = 8
home = vehicle.location.global_relative_frame
while True:
    start = time.time()
    point = target(10, -180,first_vehicle.location.global_relative_frame.alt+5)
    vehicle.simple_goto(point,6)
    time.sleep(0.5)
    distancetopoint = utils.get_distance_metres(vehicle.location.global_frame, point)
    if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
        #vehicle.simple_goto(point1)
        print("Distance to target:"+"{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
        if first_vehicle.mode == "RTL":
            print("Returning to Launch")
            vehicle.mode = VehicleMode("LAND")
            break
            # vehicle.simple_goto(home)
            # if vehicle.location.global_relative_frame == home:
            #     vehicle.mode = VehicleMode("LAND")
            #     home =0
            #     break
    #elif ord(getch.getch()) in [81,113]:
    elif first_vehicle.mode == "RTL":
        print("Returning to Launch")
        vehicle.mode = VehicleMode("LAND")
        break
        # vehicle.simple_goto(home)
        # dist = utils.get_distance_metres(vehicle.location.global_relative_frame, home)
        # print(home)
        # print(dist)
        # if dist*0.8 <= 1:
        #     vehicle.mode = VehicleMode("LAND")
        #     home =0
        #     break
            
    # elif first_vehicle.heading != vehicle.heading:
    #     condition_yaw(first_vehicle.heading)

    elif distancetopoint*0.95<=1:
        print ("Reached target")
    
    else:
        print("Change Mode Guided")
        vehicle.mode = VehicleMode("GUIDED")
    end = time.time()
    print("執行時間:"+ str(end-start))

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
first_vehicle.close()
