# -*- coding: utf-8 -*-

import json
import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)

while(1):
    data = ser.readline()
    
    try:
        data_dict = json.loads(data)
        print(data_dict)
        
    # 受信失敗時は変な型のデータが来るので除外
    except json.decoder.JSONDecodeError:
        print("JSONDecodeError")
        print(data)

    except UnicodeDecodeError:
        print("UnicodeDecodeError")
        print(data)
        
    # time.sleep(0.1)

