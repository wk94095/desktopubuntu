import threading
import time
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
from dronekit import connect, VehicleMode
import json

vehicle = connect('127.0.0.1:14561', wait_ready=True, baud=115200) #與飛機連線
vehicle1 = connect('127.0.0.1:14571', wait_ready=True, baud=115200) #與飛機連線
def aa():
    # 準備要傳送的訊息
    
    while True:
        Latitude = {'lat' : vehicle.location.global_relative_frame.lat , 'lon' : vehicle.location.global_relative_frame.lon, 'name' : 'leader'}
        Longitude = vehicle.location.global_relative_frame.lon
        messages = [
        {'topic':"drone/Latitude", 'payload': json.dumps(Latitude)},
        {'topic':"drone/Longitude", 'payload':json.dumps(Longitude), 'qos':0, 'retain':False},
        ("hello/world", "test message 2", 0, False) # topic, payload, qos, retain
        ]
        # 發布多則 MQTT 訊息
        publish.multiple(
        messages,
        hostname="192.168.0.117",
        port=1883,
        auth={'username':'bighead1','password':'nfuaesil1'})
        time.sleep(1)

def bb():
    while True:
        Latitude = {'lat' : vehicle1.location.global_relative_frame.lat , 'lon' : vehicle1.location.global_relative_frame.lon, 'name' : 'follow1'}
        
        messages = [
        {'topic':"drone1/follow", 'payload': json.dumps(Latitude)}
        
        ]
        # 發布多則 MQTT 訊息
        publish.multiple(
        messages,
        hostname="192.168.0.117",
        port=1883,
        auth={'username':'bighead1','password':'nfuaesil1'})
        time.sleep(1)

a = threading.Thread(target=aa)  # 建立新的執行緒
b = threading.Thread(target=bb)  # 建立新的執行緒

a.start()  # 啟用執行緒
b.start()  # 啟用執行緒