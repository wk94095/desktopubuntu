#!/usr/bin/env python

from dronekit import connect
import time

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200)
gimbal = vehicle.gimbal.pitch
vehicle.gimbal.rotate(-10, -60, 0)
print("set gimbal: %s" %gimbal)
time.sleep(2)
print(vehicle.gimbal)

vehicle.close()
