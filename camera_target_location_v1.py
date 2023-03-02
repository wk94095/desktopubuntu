#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Attitude
from utils.gimbalmath import target_location
from utils.drone import arm_and_takeoff
start = time.time()
vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200) #與飛機連線

#Arm and take of to altitude of 5 meters
arm_and_takeoff(vehicle, 5)
a = target_location(vehicle,30)
vehicle.simple_goto(a)
print(vehicle.location.global_relative_frame)
print(vehicle.heading)



# def xy(lat,lon,B0,L0):
#     B = lat*math.pi/180
#     L = lon*math.pi/180
#     a=6378137.0
#     b=6356752.3142
#     e=math.sqrt(1-(b/a)**2)
#     ee=math.sqrt((a/b)**2-1)
#     K=(((a**2*math.cos(B0))/b)/math.sqrt(1+(ee)**2*(math.cos(B0))**2))
#     X=K*(L-L0)
#     Y=K*math.log(math.tan(math.pi/4+B/2)*((1-e*math.sin(B))/(1+e*math.sin(B)))**(e/2))
#     return X,Y
# print(xy(23.7025595, 120.4230539, 0, 0))

vehicle.close()
end = time.time()

print("執行時間: %f 秒" %(end-start))