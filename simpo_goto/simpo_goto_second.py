#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
import getch #偵測鍵盤按鍵
import utils

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
vehicle1 = connect('127.0.0.1:14581', wait_ready=True, baud=115200) #與飛機連線

def arm_and_takeoff(aTargetAltitude): #定義起飛程序
    """
    Arms vehicle and fly to aTargetAltitude.
    """

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
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2): #定義目標位置與目標位置計算出距離
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

if vehicle.armed != True:
    arm_and_takeoff(8) #起飛高度
    print("takeoff")
else:
    vehicle.mode = VehicleMode("GUIDED")
    print("change mode")

print("Set default/target airspeed to 8")
vehicle.airspeed = 8

#print("Going towards first point for 30 seconds ...")
while True:
    #print("繼續執行或按q退出")
    lat = vehicle1.location.global_relative_frame.lat #抓取掌機緯度座標
    #print(lat)
    lon = vehicle1.location.global_relative_frame.lon #抓取掌機經度座標
    #print(lon)
    lat = float(lat) #轉換為浮點數
    lon = float(lon) #轉換為浮點數
    point1 = LocationGlobalRelative(lat, lon, 12)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, point1)
    print("{:.2f}".format(distancetopoint)) #{}內容會讀取後面.format內的值，如{:.3f}表示將distancetopoint填充到槽中時，取小數點後3位
    time.sleep(1)
    if distancetopoint >=1: #離目標距離大於1時會繼續往目標前進，直到小於1時跳出
        vehicle.simple_goto(point1)
    #elif ord(getch.getch()) in [81,113]:
    elif vehicle1.mode == "RTL":
        print("Returning to Launch")
        vehicle.mode = VehicleMode("RTL")
        break
    else:
        print("Change Mode Guided")
        vehicle.mode = VehicleMode("GUIDED")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()
vehicle1.close()
