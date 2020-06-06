# -*- coding: utf-8 -*-

from math import *
import smbus
import time
import Kalman as kf

# some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
    # Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Set Gyro Full Scale Range to ±250deg/
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 0)

    # Set Accelerometer Full Scale Range to ±2g
    bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0)

    # PLL with X axis gyroscope reference and disable sleep mode
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    
    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68 # MPU6050 device address

MPU_Init()

time.sleep(0.1)

RAD_TO_DEG = 180/pi

kalmanX = kf.Kalman()
kalmanY = kf.Kalman()
kalmanZ = kf.Kalman()

accX = read_raw_data(ACCEL_XOUT_H)
accY = read_raw_data(ACCEL_YOUT_H)
accZ = read_raw_data(ACCEL_ZOUT_H)

roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG
pitch = atan2(-accX, accZ) * RAD_TO_DEG

kalmanX.setAngle(roll) # Set starting angle
kalmanY.setAngle(pitch)
gyroXangle = roll
gyroYangle = pitch
compAngleX = roll
compAngleY = pitch
kalAngleX = roll
kalAngleY = pitch

# タイマ割り込みの周期[s]
dt = 1.0e-4

def kalmanLoop(kalAngleX, kalAngleY, kalmanX, kalmanY, kalmanZ, gyroXangle, gyroYangle, compAngleX, compAngleY):
    # Update all the values
    accX = read_raw_data(ACCEL_XOUT_H)
    accY = read_raw_data(ACCEL_YOUT_H)
    accZ = read_raw_data(ACCEL_ZOUT_H)
    tempRaw = read_raw_data(TEMP_OUT_H)
    gyroX = read_raw_data(GYRO_XOUT_H)
    gyroY = read_raw_data(GYRO_YOUT_H)
    gyroZ = read_raw_data(GYRO_ZOUT_H)

    roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG
    pitch = atan2(-accX, accZ) * RAD_TO_DEG

    gyroXrate = gyroX / 131.0 # Convert to deg/s
    gyroYrate = gyroY / 131.0 # Convert to deg/s

    # This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if((pitch < -90 and kalAngleY > 90) or (pitch > 90 and kalAngleY < -90)):
        kalmanY.setAngle(pitch)
        compAngleY = pitch
        kalAngleY = pitch
        gyroYangle = pitch
    else:
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt) # Calculate the angle using a Kalman filter

    if(abs(kalAngleY) > 90):
        gyroXrate = -gyroXrate # Invert rate, so it fits the restriced accelerometer reading
        
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt) # Calculate the angle using a Kalman filter

    gyroXangle += gyroXrate * dt # Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt
    # gyroXangle += kalmanX.getRate() * dt # Calculate gyro angle using the unbiased rate
    # gyroYangle += kalmanY.getRate() * dt

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll # Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch

    # Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 or gyroXangle > 180):
        gyroXangle = kalAngleX
    if (gyroYangle < -180 or gyroYangle > 180):
        gyroYangle = kalAngleY

    print("pitch:{}, gy_pitch:{}, cmp_pitch:{}, kal_pitch:{}".format(pitch, gyroYangle, compAngleY, kalAngleY))

    return kalAngleX, kalAngleY, kalmanX, kalmanY, kalmanZ, gyroXangle, gyroYangle, compAngleX, compAngleY

while(1):
    kalAngleX, kalAngleY, kalmanX, kalmanY, kalmanZ, gyroXangle, gyroYangle, compAngleX, compAngleY = kalmanLoop(kalAngleX, kalAngleY, kalmanX, kalmanY, kalmanZ, gyroXangle, gyroYangle, compAngleX, compAngleY)

#   /* Print Data */
# #if 0 // Set to 1 to activate
#   Serial.print(accX); Serial.print("\t");
#   Serial.print(accY); Serial.print("\t");
#   Serial.print(accZ); Serial.print("\t");

#   Serial.print(gyroX); Serial.print("\t");
#   Serial.print(gyroY); Serial.print("\t");
#   Serial.print(gyroZ); Serial.print("\t");

#   Serial.print("\t");
# #endif

#   Serial.print(roll); Serial.print("\t");
#   Serial.print(gyroXangle); Serial.print("\t");
#   Serial.print(compAngleX); Serial.print("\t");
#   Serial.print(kalAngleX); Serial.print("\t");

#   Serial.print("\t");

#   Serial.print(pitch); Serial.print("\t");
#   Serial.print(gyroYangle); Serial.print("\t");
#   Serial.print(compAngleY); Serial.print("\t");
#   Serial.print(kalAngleY); Serial.print("\t");

# #if 0 // Set to 1 to print the temperature
#   Serial.print("\t");

#   double temperature = (double)tempRaw / 340.0 + 36.53;
#   Serial.print(temperature); Serial.print("\t");
# #endif

#   Serial.print("\r\n");
#   delay(2);
# }