# -*- coding: utf-8 -*-

from math import *
import smbus
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

def kalmanLoop():
    # Update all the values
    accX = read_raw_data(ACCEL_XOUT_H)
    accY = read_raw_data(ACCEL_YOUT_H)
    accZ = read_raw_data(ACCEL_ZOUT_H)
    tempRaw = read_raw_data()
#   gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
#   gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
#   gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

#   double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
#   timer = micros();

#   // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
#   // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
#   // It is then converted from radians to degrees
# #ifdef RESTRICT_PITCH // Eq. 25 and 26
#   double roll  = atan2(accY, accZ) * RAD_TO_DEG;
#   double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
# #else // Eq. 28 and 29
#   double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#   double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
# #endif

#   double gyroXrate = gyroX / 131.0; // Convert to deg/s
#   double gyroYrate = gyroY / 131.0; // Convert to deg/s

# #ifdef RESTRICT_PITCH
#   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
#   if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
#     kalmanX.setAngle(roll);
#     compAngleX = roll;
#     kalAngleX = roll;
#     gyroXangle = roll;
#   } else
#     kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

#   if (abs(kalAngleX) > 90)
#     gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
#   kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
# #else
#   // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
#   if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
#     kalmanY.setAngle(pitch);
#     compAngleY = pitch;
#     kalAngleY = pitch;
#     gyroYangle = pitch;
#   } else
#     kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

#   if (abs(kalAngleY) > 90)
#     gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
#   kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
# #endif

#   gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
#   gyroYangle += gyroYrate * dt;
#   //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
#   //gyroYangle += kalmanY.getRate() * dt;

#   compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
#   compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

#   // Reset the gyro angle when it has drifted too much
#   if (gyroXangle < -180 || gyroXangle > 180)
#     gyroXangle = kalAngleX;
#   if (gyroYangle < -180 || gyroYangle > 180)
#     gyroYangle = kalAngleY;

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
