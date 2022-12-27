from dronekit import connect
from pymavlink import mavutil
import time

vehicle = connect('127.0.0.1:14560', wait_ready=True, baud=115200)
first_vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=115200) #與飛機連線

def send_global_ned_velocity(x, y, z):
    msg = first_vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b110111111000,
        x,y,z,
        0,0,0,
        0,0,0,
        0,0
        )
    vehicle.send_mavlink(msg)
    print(msg)
    vehicle.flush()

send_global_ned_velocity(10,0,0)

vehicle.close()
first_vehicle.close()