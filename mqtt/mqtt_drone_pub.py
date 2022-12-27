import paho.mqtt.client as mqtt
import random
import json  
import datetime 
import time
from dronekit import connect

vehicle = connect("127.0.0.1:14560", wait_ready=True, baud=115200)
first_vehicle = connect("127.0.0.1:14550", wait_ready=True, baud=115200)

# 設置日期時間的格式
ISOTIMEFORMAT = '%m/%d %H:%M:%S'

# 連線設定
# 初始化地端程式
client = mqtt.Client()

# 設定登入帳號密碼
client.username_pw_set("bighead","nfuaesil")

# 設定連線資訊(IP, Port, 連線時間)
client.connect("192.168.0.132", 1883, 60)

while True:
    t0 = random.randint(0,30)
    t = datetime.datetime.now().strftime(ISOTIMEFORMAT)
    latitude = vehicle.location.global_relative_frame.lat
    first_latitude = first_vehicle.location.global_relative_frame.lat

    payload = {'Temperature' : t0 , 'Time' : t}
    payload_lat = {'Latitude' : latitude}
    payload_firstlat = {'Latitude1' : first_latitude}
    print (json.dumps(payload_lat))
    print (json.dumps(payload_firstlat))
    #要發布的主題和內容
    client.publish("drone/Latitude", json.dumps(payload_lat))
    client.publish("drone/Latitude1", json.dumps(payload_firstlat))
    time.sleep(1)

vehicle.close()