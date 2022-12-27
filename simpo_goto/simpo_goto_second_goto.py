#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
import getch #偵測鍵盤按鍵
import utils
from pymavlink import mavutil

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
vehicle1 = connect('127.0.0.1:14550', wait_ready=True, baud=115200) #與飛機連線

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

def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2): #定義目標位置與目標位置計算出距離
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    global remainingDistance, targetDistance

    currentLocation=vehicle1.location.global_relative_frame #當前位置
    targetLocation=get_location_metres(currentLocation, dNorth, dEast)
    targetDistance=get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    # while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
    remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
    #     print ("Distance to target: ", remainingDistance)
    #     if remainingDistance<=targetDistance*0.1: #Just below target, in case of undershoot.
    #         print ("Reached target")
    #         break
    #     time.sleep(2)

def aux(ACTUATOR,pwm):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        ACTUATOR,pwm,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

if vehicle.armed != True:
    arm_and_takeoff(20) #起飛高度
    print("takeoff")
else:
    vehicle.mode = VehicleMode("GUIDED")
    print("change mode")

print("Set default/target airspeed to 8")
vehicle.airspeed = 8

while True:

    lat = vehicle1.location.global_relative_frame.lat #讀取掌機緯度座標
    lon = vehicle1.location.global_relative_frame.lon #讀取掌機經度座標
    lat = float(lat) #轉換為浮點數
    lon = float(lon) #轉換為浮點數
    point1 = LocationGlobalRelative(lat, lon, 40)
    point2 = goto(20,0)
    time.sleep(2)
    if remainingDistance >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
        #point2
        print("Distance to target:"+"{:.2f}".format(remainingDistance)) #{}內容會讀取後面.format內的值，如{:.3f}表示將remainingDistance填充到槽中時，取小數點後3位
        if vehicle1.mode == "RTL":
            print("Returning to Launch")
            vehicle.mode = VehicleMode("RTL")
            break
    #elif ord(getch.getch()) in [81,113]:
    elif vehicle1.mode == "RTL":
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        break
    elif remainingDistance<=targetDistance*0.1:
        print ("Reached target")
        aux(10,1900)
        continue
    else:
        print("Change Mode Guided")
        vehicle.mode = VehicleMode("GUIDED")  

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
vehicle1.close()
