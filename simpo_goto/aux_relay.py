#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, LocationLocal
import math
#from pymavlink import mavutil
import utils

vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=115200) #與飛機連線


# for _ in range(3):
#     utils.relay(vehicle,0,1)
#     utils.aux(vehicle,12,900)
#     print(" Ch12:900" )
#     time.sleep(1)
#     utils.relay(vehicle,0,0)
#     utils.aux(vehicle,12,1900)
#     print(" Ch12:1900")
#     time.sleep(1)

# #relay(0,1)
# print("OPEN")
# time.sleep(3)
# utils.aux(vehicle,12,900)
# #relay(0,0)
# print("CLOSE")
# time.sleep(1)
# utils.aux(vehicle,12,1900)



print("Close vehicle object")
vehicle.close()
