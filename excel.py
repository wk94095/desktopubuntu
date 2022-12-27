import socket
import time
import datetime
import pandas as pd
import math
from dronekit import connect

vehicle = connect('127.0.0.1:14560',rate=1, wait_ready=True, baud=115200)

hostname = socket.gethostname()
ip=socket.gethostbyname(hostname)
ISOTIMEFORMAT = '%m/%d %H:%M:%S'
ISOTIMEFORMAT1 = '%m-%d-%H-%M'
t = datetime.datetime.now().strftime(ISOTIMEFORMAT)
t1 = datetime.datetime.now().strftime(ISOTIMEFORMAT1)
pitch = math.degrees(vehicle.attitude.pitch)
roll = math.degrees(vehicle.attitude.roll)
airspeed = vehicle.airspeed
print(pitch,roll,airspeed)

mid_term_marks = {"pitch": [pitch],
                  "roll": [roll],
                  "airspeed" : [airspeed],
                  "Time" : [t]
                  }

mid_term_marks_df = pd.DataFrame(mid_term_marks)

mid_term_marks_df.to_csv("csv/"+str(t1)+".csv", index=False, encoding='utf_8_sig')

while True:
    if vehicle.mode.name !="RTL":
        pitch = math.degrees(vehicle.attitude.pitch)
        roll = math.degrees(vehicle.attitude.roll)
        airspeed = vehicle.airspeed
        print(pitch,roll,airspeed)
        t = datetime.datetime.now().strftime(ISOTIMEFORMAT)
        mid_term_marks = pd.DataFrame([[pitch,roll,airspeed,t]])
        mid_term_marks.to_csv("csv/"+str(t1)+".csv", mode='a', header=False,index=False, encoding='utf_8_sig')
        time.sleep(1)
    else:
        break

vehicle.close()
