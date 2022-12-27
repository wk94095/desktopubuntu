#!/usr/bin/env python

from dronekit import connect
import time

vehicle = connect('192.168.0.119:14551', wait_ready=True, baud=115200)

for _ in range(10):
    #print(vehicle.gps_time.gps_time)
    #"Latitude, Longtitude, Altitude"
    print(vehicle.location.global_relative_frame)
    #D4, D5, D6
    print(vehicle.attitude)
    #D1, D2, D3
    print(vehicle.velocity)
    #V, A, BatteryStatus
    print(vehicle.battery)
    #AirSpeed
    print(vehicle.airspeed)
    #GroundSpeed
    print(vehicle.groundspeed)
    #CurrentFlightMode
    print(vehicle.mode.name)
    #GPSStatus
    print(vehicle.gps_0)
    #Motor (report armed/disarmed)
    print(vehicle.armed)
    #waypointnumber
    #print("waypoint %s " % vehicle.commands.next)
    #print(vehicle.capabilities.flight_termination)
    #print(dronetime(vehicle))
    print("="*100)
    time.sleep(1)

vehicle.close()
