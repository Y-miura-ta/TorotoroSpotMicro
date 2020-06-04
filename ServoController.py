# -*- coding: utf-8 -*-

import numpy as np
from math import *
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# lf, rf, lb, rb
SERVO_NUM = 12
max_pulse = np.array([
    2658,2255,2560,
    2658,2577,2560,
    2658,2255,2560,
    2658,2577,2560
])
min_pulse = np.array([
    324,319,577,
    324,641,577,
    324,319,577,
    324,641,577
])
deg_center_pos = np.array([
    92,92,90+90,
    94,101,104-90,
    95,94,83+90,
    92,94,104-90,
])
IK_sign = np.array([
    -1, -1, -1,
    1, 1, 1,
    -1, -1, -1,
    1, 1, 1    
])
pulse_to_deg = 180/(max_pulse - min_pulse)
deg_to_pulse = (max_pulse - min_pulse)/180

servos = [servo.Servo(pca.channels[i], min_pulse=min_pulse[i], max_pulse=max_pulse[i]) for i in range(SERVO_NUM)]

# degreeの角度をセット
def setDegThetas(deg_thetas):
    deg_pos = deg_center_pos +IK_sign*deg_thetas 
    for i in range(SERVO_NUM):
        servos[i].angle = deg_pos[i]

# ラジアンの角度をセット
def setRadThetas(rad_thetas):
    deg_pos = deg_center_pos +IK_sign*(180*rad_thetas/pi) 
    for i in range(SERVO_NUM):
        servos[i].angle = deg_pos[i]

def calibrateServo():
    print("Input servo ID to calibrate")
    calib_servo_ID = int(input())
    _deg_to_pulse = deg_to_pulse[calib_servo_ID]

    print("Input start degree")
    start_deg = float(input())

    print("Input degree stride")
    stride_deg = float(input())

    set_deg = start_deg

    while(1):
        print("Input j or k")
        key = input()
        if(key == 'j'):
            set_deg -= stride_deg
        elif(key == 'k'):
            set_deg += stride_deg
        elif(key == 'q'):
            print("Quit")
            print("--------------------------------------------------")
            print()
            break
        
        servos[calib_servo_ID].angle = set_deg
        print("servo:{}, deg:{}".format(calib_servo_ID, set_deg))
        print("pulse:{}".format(int(min_pulse[calib_servo_ID]+set_deg*_deg_to_pulse)))
        print("--------------------------------------------------")

def main():
    for i in range(SERVO_NUM):
        servos[i].angle = deg_center_pos[i]
    # キャリブレーション
    while(1):
        calibrateServo()
    
if __name__ == '__main__':
    main()
