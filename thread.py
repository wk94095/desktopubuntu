from dronekit import connect, VehicleMode
import threading
from utils.drone import aux
import time

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200)

def data():
    while True:
        print(vehicle.location.global_relative_frame)
        #D4, D5, D6
        print(vehicle.attitude)
        #D1, D2, D3
        print(vehicle.velocity)
        #V, A, BatteryStatus
        print(vehicle.battery)
        #AirSpeed
        print(vehicle.airspeed)
        time.sleep(1)

def aux1():
    while True:
        aux(vehicle,12,900)
        print('ch12:900')
        time.sleep(1)
        aux(vehicle,12,1900)
        print('ch12:1900')
        time.sleep(1)

p1 = threading.Thread(target=data)
p2 = threading.Thread(target=aux1)
p1.start()
p2.start()