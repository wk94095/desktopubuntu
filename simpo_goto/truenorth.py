#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, LocationLocal
import math
import getch #偵測鍵盤按鍵
import utils
from pymavlink import mavutil

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
vehicle1 = connect('127.0.0.1:14550', wait_ready=True, baud=115200) #與飛機連線

# print(vehicle.location.global_relative_frame)
# print(vehicle.location.local_frame)

# print(vehicle.heading)
#print(vehicle.location.local_frame)

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

def send_global_ned_velocity(x, y, z):
    msg = vehicle1.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111000111,
        0,0,0,
        x,y,z,
        0,0,0,
        0,0
        )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def aux(ACTUATOR,pwm):
    # create the MAV_CMD_DO_SET_ROI command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.RC_CHANNELS_SCALED, #command
        0, #confirmation
        ACTUATOR,pwm,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def relay(Instance,setting):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY, #command
        0, #confirmation
        Instance,setting,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def message1(test1):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL  , #command
        0, #confirmation
        test1,0,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    print(msg)
    vehicle.flush()

send_global_ned_velocity(10,10,0)
vehicle.close()
vehicle1.close()
