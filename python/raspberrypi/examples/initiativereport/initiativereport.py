# -*- coding: utf-8 -*
'''
  * @file  initiativereport.ino
  * @brief �����������ϱ�ȫ������
  * @n ʵ�鷽ʽ�� ��������ͨ���������������ӣ���¼
  * @n ͨ�ŷ�ʽѡ�񣬲��뿪��SEL��0��IIC,1��UART
  * �����         ���ڵ�ַ
  A0 A1�����ƽ    00    01    10    11
    1            0x60  0x61  0x62  0x63
    2            0x64  0x65  0x66  0x67
    3            0x68  0x69  0x6A  0x6B
    4            0x6C  0x6D  0x6E  0x6F
    5            0x70  0x71  0x72  0x73
    6��Ĭ�ϵ�ַ�飩 0x74  0x75  0x76  0x77��Ĭ�ϵ�ַ��
    7            0x78  0x79  0x7A  0x7B
    8            0x7C  0x7D  0x7E  0x7F
    @n i2c ��ַѡ��Ĭ��i2c��ַΪ0x77��A1��A0��ϳ�4��IIC��ַ
                | A0 | A1 |
                | 0  | 0  |    0x74
                | 0  | 1  |    0x75
                | 1  | 0  |    0x76
                | 1  | 1  |    0x77   default i2c address    
  * @n ʵ������ ���ڴ�ӡȫ������
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V1.0
  * @date        2021-03-28
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/dfrobot/DFRobot_MultiGasSensor
'''

import sys
import os
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
from DFRobot_MultiGasSensor import *

'''
  ctype=1��UART
  ctype=0��IIC
'''
ctype=1

if ctype==0:
  I2C_1       = 0x01               # I2C_1 ʹ��i2c1�ӿ������������� ���Ե���Ϊi2c0������Ҫ������ݮ�ɵ��ļ�
  I2C_ADDRESS = 0x77               # I2C �豸�ĵ�ַ�����Ը���A1��A0��������ַ��Ĭ�ϵ�ַΪ0x77
  gas = DFRobot_MultiGasSensor_I2C(I2C_1 ,I2C_ADDRESS)
else:
  gas = DFRobot_MultiGasSensor_UART(9600)

def setup():
  #��ȡ����ģʽΪ�������������ϱ�����
  gas.change_acquire_mode(gas.INITIATIVE)
  time.sleep(1)
def loop():
  if(True==gas.data_is_available()):
    print("========================")
    print("gastype:"+str(gas.gastype))
    print("------------------------")
    print("gasconcentration:"+str(round(gas.gasconcentration,3))+"%")
    print("------------------------")
    print("temp:"+str(round(gas.temp,3))+" C")
    print("========================")
  #time.sleep(1)

if __name__ == "__main__":
  setup()
  while(1):
    loop()
  