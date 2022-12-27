from dronekit import connect
import math
import time
from pymavlink import mavutil
from utils.drone import *

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200)

def aux(ACTUATOR,pwm): #設定aux通道
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        ACTUATOR,pwm,0,0,0,0,0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

while True:
    att_pitch = math.degrees(vehicle.attitude.pitch)
    airspeed = vehicle.airspeed
    if att_pitch <-1 and airspeed <1:
        send_global_velocity(vehicle,1,0,0,1)
        time.sleep(1)
        print("front")
        if airspeed <1:
            time.sleep(1)
            print(airspeed)
            for _ in range(3):
                aux(12,1900)
                print('open')
                time.sleep(1)
                aux(12,1000)
                print('close')
                time.sleep(1)
            
    else:
        print("hello")
        print(airspeed)
        time.sleep(1)

vehicle.close()

