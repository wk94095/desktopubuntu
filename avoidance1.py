from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,LocationGlobal
import math
from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json
import paho.mqtt.publish as publish
import threading

vehicle = connect('127.0.0.1:14570', wait_ready=True, baud=115200) #與飛機連線
first_vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
flag = 0

def arm_and_takeoff(aTargetAltitude): #定義起飛程序
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # while not first_vehicle.armed:
    #     print("waiting for leader arming")
    #     time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    # first_vehicle.mode = VehicleMode("GUIDED")
    # first_vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    #first_vehicle.simple_takeoff(aTargetAltitude)
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

def get_location_metres(original_location, dNorth, dEast, dAlt):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    newalt = dAlt
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon, newalt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon, newalt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def get_distance_metres(aLocation1, aLocation2): #定義目標位置與目標位置計算出距離
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    dalt = aLocation2.alt - aLocation1.alt
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5 + math.sqrt(dalt*dalt)

def send_global_ned_velocity(x, y, z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,0,0,
        x,y,z,
        0,0,0,
        0,0
        )
    vehicle.send_mavlink(msg)
    #print(msg)
    vehicle.flush()
arm_and_takeoff(10)
#get_distance_metres(follower,leader)
while True:
    leader = first_vehicle.location.global_relative_frame
    follower = vehicle.location.global_relative_frame
    dis = get_distance_metres(follower,leader)
    dle = first_vehicle.location.global_relative_frame.alt - vehicle.location.global_relative_frame.alt
    print(dis)
    if dis <2:
        send_global_ned_velocity(0,0,-1)
        print("vehicle:"+"{:.2f}".format(vehicle.location.global_relative_frame.alt))
    time.sleep(1)

        
    