# -*- coding: utf-8 -*
'''
  @file  set_threshold_alarm.py
  @brief 设置传感器的阈值报警
  @n 实验方式： 将传感器通信引脚与主控连接，烧录，将BCM的18号引脚与传感器的ALA引脚连接
  @n 通信方式选择，拨码开关SEL：0：IIC,1：UART
  @n 组序号         组内地址
  @n A0 A1拨码电平 00    01    10    11
  @n 1            0x60  0x61  0x62  0x63
  @n 2            0x64  0x65  0x66  0x67
  @n 3            0x68  0x69  0x6A  0x6B
  @n 4            0x6C  0x6D  0x6E  0x6F
  @n 5            0x70  0x71  0x72  0x73
  @n 6（默认地址组） 0x74  0x75  0x76  0x77（默认地址）
  @n 7            0x78  0x79  0x7A  0x7B
  @n 8            0x7C  0x7D  0x7E  0x7F
  @n @n i2c 地址选择，默认i2c地址为0x77，A1、A0组合成4种IIC地址
  @n             | A0 | A1 |
  @n             | 0  | 0  |    0x74
  @n             | 0  | 1  |    0x75
  @n             | 1  | 0  |    0x76
  @n             | 1  | 1  |    0x77   default i2c address   
  @n 实验现象： 当传感器获取到的数据值超过设置的阈值之后，传感器的ALA引脚将输出高电平
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      PengKaixing(kaixing.peng@dfrobot.com)
  @version     V2.0
  @date        2021-03-28
  @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
'''

import sys
import os
import time
import RPi.GPIO as GPIO

#报警引脚
pin = 18

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
from DFRobot_MultiGasSensor import *

'''
  ctype=1：UART
  ctype=0：IIC
'''
ctype=1

if ctype==0:
  I2C_1       = 0x01               # I2C_1 使用i2c1接口驱动传感器， 可以调整为i2c0但是需要配置树莓派的文件
  I2C_ADDRESS = 0x77               # I2C 设备的地址，可以更改A1、A0来更换地址，默认地址为0x77
  gas = DFRobot_MultiGasSensor_I2C(I2C_1 ,I2C_ADDRESS)
else:
  gas = DFRobot_MultiGasSensor_UART(9600)

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin,GPIO.IN)

def setup():
  while (False==gas.set_threshold_alarm(gas.ON,20,read_gas_type)):
    print ("set alarm ERROR!")
    time.sleep(1)
  
def loop():
  gas.read_gas_concentration()
  if GPIO.input(pin)==0:
    print ("warning!!!")
  else:
    print ("nolmal!!!")
  time.sleep(1)  

if __name__ == "__main__":
  setup()
  while True:
    loop()