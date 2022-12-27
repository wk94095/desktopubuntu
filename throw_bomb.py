from dronekit import connect
from utils.drone import arm_and_takeoff, aux, relay

vehicle = connect('127.0.0.1:14550', wait_ready=True, baud=115200)

arm_and_takeoff(vehicle,10)
import paho.mqtt.client as mqtt
import time
import json

subscribe_topic_running = "target/position"

# 當地端程式連線伺服器得到回應時，要做的動作
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # 將訂閱主題寫在on_connet中
    # 如果我們失去連線或重新連線時 
    # 地端程式將會重新訂閱
    client.subscribe("drone/throw")

# 當接收到從伺服器發送的訊息時要進行的動作
def on_message(client, userdata, msg):
    # 轉換編碼utf-8才看得懂中文
    print(msg.topic+" "+ msg.payload.decode('utf-8'))
    #a = json.loads(msg.payload.decode('utf-8'))
    b = json.loads(msg.payload.decode('utf-8'))
    payload = {"throw" : 'off'}
    throw = b["throw"]
    print(b)
    if throw == "on":
        aux(vehicle,12,1900)
        print("on")
        time.sleep(1)
        aux(vehicle,12,1000)
        print("off")
        client.publisher('drone/throw', json.dumps(payload))
    # if a.get("mode")==1:
    #     print(a.get("mode"))
    # elif a.get("Temperature") >0:
    #     print(a.get("Temperature"))
    #print(a.get("Temperature"),a.get("Time"))
    #print(a.get("Time"))

# 連線設定
# 初始化地端程式
client = mqtt.Client()

# 設定連線的動作
client.on_connect = on_connect

# 設定接收訊息的動作
client.on_message = on_message
#client.message_callback_add(subscribe_topic_running, on_running_message)
# 設定登入帳號密碼
client.username_pw_set("bighead","nfuaesil")

# 設定連線資訊(IP, Port, 連線時間)
client.connect("192.168.0.117", 1883, 60)

# 開始連線，執行設定的動作和處理重新連線問題
# 也可以手動使用其他loop函式來進行連接
client.loop_forever()





vehicle.close()