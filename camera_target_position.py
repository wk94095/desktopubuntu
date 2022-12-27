#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import math

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

#Arm and take of to altitude of 5 meters
arm_and_takeoff(5)
lat = vehicle.location.global_relative_frame.lat #無人機緯度座標
lon = vehicle.location.global_relative_frame.lon #無人機經度座標
alt = vehicle.location.global_relative_frame.alt
gimbal_angle = math.radians(10)
droneheading = math.radians(90-vehicle.heading)
print(droneheading)
height = 100
r=200
distance = height*math.tan(gimbal_angle)
print("離目標物距離:",distance)

earth_radius=6378137.0

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

def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

#print(get_location_metres(vehicle.location.global_relative_frame, 10 ,10))
print(vehicle.location.global_relative_frame)
print(vehicle.heading)
point1 = LocationGlobalRelative(newlat, newlon, alt)
vehicle.simple_goto(point1)
print("====================================")

def xy(lat,lon,B0,L0):
    B = lat*math.pi/180
    L = lon*math.pi/180
    a=6378137.0
    b=6356752.3142
    e=math.sqrt(1-(b/a)**2)
    ee=math.sqrt((a/b)**2-1)
    K=(((a**2*math.cos(B0))/b)/math.sqrt(1+(ee)**2*(math.cos(B0))**2))
    X=K*(L-L0)
    Y=K*math.log(math.tan(math.pi/4+B/2)*((1-e*math.sin(B))/(1+e*math.sin(B)))**(e/2))
    return X,Y
    

# print(xy(23.7025595, 120.4230539, 0, 0))

vehicle.close()