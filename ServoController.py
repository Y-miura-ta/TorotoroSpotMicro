# -*- coding: utf-8 -*-

import numpy as np
from math import *
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
# from pynput import keyboard

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

SERVO_NUM = 12
max_pulse = [
    452,2000,2000,
    2000,2000,2000,
    2000,2000,2000,
    2000,2000,2000
]
min_pulse = [
    2560,1000,1000,
    1000,1000,1000,
    1000,1000,1000,
    1000,1000,1000
]
pulse_center_pos = [
    2560,1500,1500,
    1500,1500,1500,
    1500,1500,1500,
    1500,1500,1500
]
deg_center_pos = 180*(np.array(pulse_center_pos)-np.array(min_pulse))/(np.array(max_pulse)-np.array(min_pulse))
pulse_to_deg = 180/(np.array(max_pulse)-np.array(min_pulse))
deg_to_pulse = (np.array(max_pulse)-np.array(min_pulse))/180

servos = [servo.Servo(pca.channels[i], min_pulse=min_pulse[i], max_pulse=max_pulse[i]) for i in range(SERVO_NUM)]

# キーボード監視用
# def on_press(key):
#     try:
#         print('alphanumeric key {0} pressed'.format(key.char))
#     except AttributeError:
#         print('special key {0} pressed'.format(key))

# def on_release(key):
#     print('{0} released'.format(key))
#     if key == keyboard.Key.esc:
#         return False

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
    #key_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    #key_listener.start()

def main():
    # キャリブレーション
    while(1):
        calibrateServo()
    
if __name__ == '__main__':
    main()