# -*- coding: utf-8 -*-x

import smbus
import time

# device address
DEV_ADDR = 0x68
# register address
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b       # PWR_MGMT_1
PWR_MGMT_2 = 0x6c       # PWR_MGMT_2

bus = smbus.SMBus(1)

bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

# 2byte read
def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val

# Sensor data read
def read_word_sensor(adr):
    val = read_word(adr)
    # minus
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    # plus
    else:
        return val
    
# temperture
def get_temp():
    temp = read_word_sensor(TEMP_OUT)
    x = temp / 340 + 36.53
    return x

# gyro data
# full scale range ±250 deg/s LSB sensitivity 131 LSB/deg/s -> ±250 x 131 = ±32750 LSB
def get_gyro_data_deg():
    x = read_word_sensor(GYRO_XOUT)
    y = read_word_sensor(GYRO_YOUT)
    z = read_word_sensor(GYRO_ZOUT)

    x = x / 131.0
    y = y / 131.0
    z = z / 131.0
    return [x, y, z]

# accel data
# full scale range ±2g LSB sensitivity 16384 LSB/g) -> ±2 x 16384 = ±32768 LSB
def get_accel_data_g():
    x = read_word_sensor(ACCEL_XOUT)
    y = read_word_sensor(ACCEL_YOUT)
    z = read_word_sensor(ACCEL_ZOUT)

    x = x / 16384.0
    y = y / 16384.0
    z = z / 16384.0
    return [x, y, z]

while 1:
    
    temp = get_temp()
    gyro_x,gyro_y,gyro_z = get_gyro_data_deg()
    accel_x,accel_y,accel_z = get_accel_data_g()

    print ('%f[deg C] gyro[deg/s] x:%f y:%f z:%f accel[g] x:%f y:%f z:%f' % (temp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z))
    # send_s = str(int(temp*100))+','+str(int(gyro_x*100))+','+str(int(gyro_y*100))+','+str(int(gyro_z*100))+','+str(int(accel_x*100))+','+str(int(accel_y*100))+','+str(int(accel_z*100))+',\n'
    # print(send_s)
    # s.write(bytes(send_s,'UTF-8'))

    time.sleep(0.05)

    xGyroValue = float(splitdata[1])/100*3.142/180;
    yGyroValue = float(splitdata[2])/100*3.142/180;
    zGyroValue = float(splitdata[3])/100*3.142/180;
    xAxisValue = float(splitdata[4])/100;
    yAxisValue = float(splitdata[5])/100;
    zAxisValue = float(splitdata[6])/100;
    
    println(xGyroValue);
    println(yGyroValue);
    println(zGyroValue);
    println(xAxisValue);
    println(yAxisValue);
    println(zAxisValue);
  
  // 角速度を積分して角度に変換
  zGyroAngleValue = zGyroAngleValue + zGyroValue*0.05;
  
  // 加速度から角度を計算
  float xAxisAngleValue = asin(yAxisValue/zAxisValue);
  float yAxisAngleValue = asin(xAxisValue/zAxisValue);
  
  xAngleValue = 0.98*(xAngleValue + xGyroValue*0.05)+0.02*xAxisAngleValue;
  yAngleValue = 0.98*(yAngleValue - yGyroValue*0.05)+0.02*yAxisAngleValue;
  
  println(xAngleValue);
  println(yAngleValue);
  println(zGyroAngleValue);
    
#s.close()
