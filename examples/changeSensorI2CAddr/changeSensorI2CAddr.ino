/*!
  * @file  changeSensorI2CAddr.ino
  * @brief 传感器主动上报全部数据，仅限iic通信使用这个demo
  * @n 实验方式： 将传感器通信引脚与主控连接，烧录
  * @n 通信方式选择，拨码开关SEL：0：IIC,1：UART
  * 组序号         组内地址
  A0 A1拨码电平    00    01    10    11
    1            0x60  0x61  0x62  0x63
    2            0x64  0x65  0x66  0x67
    3            0x68  0x69  0x6A  0x6B
    4            0x6C  0x6D  0x6E  0x6F
    5            0x70  0x71  0x72  0x73
    6（默认地址组） 0x74  0x75  0x76  0x77（默认地址）
    7            0x78  0x79  0x7A  0x7B
    8            0x7C  0x7D  0x7E  0x7F
    @n i2c 地址选择，默认i2c地址为0x77，A1、A0组合成4种IIC地址
                | A0 | A1 |
                | 0  | 0  |    0x74
                | 0  | 1  |    0x75
                | 1  | 0  |    0x76
                | 1  | 1  |    0x77   default i2c address   
  * @n 实验现象： 串口打印全部数据
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V1.0
  * @date        2021-03-28
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/dfrobot/DFRobot_MultiGasSensor
*/
#include "DFRobot_MultiGasSensor.h"

#define I2C_ADDRESS    0x77
  DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);

uint8_t scan()
{
  Wire.begin();
  for(uint8_t address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
      return address;
  }
  return 0xff;
}
void setup() {
  Serial.begin(115200);
  gas.setI2cAddr(scan());
/**
  传感器初始化，用作初始化串口或者初始化IIC，由此时使用的通信方式来决定
*/
  while(!gas.begin())
  {
    Serial.println("NO Deivces !");
    delay(1000);
  }

/**
  修改iic地址组
*/
  while(gas.changeI2cAddrGroup(3)==0)
  {
    Serial.println("IIC addr change fail!");
    delay(1000);
  }  
  Serial.println("IIC addr change success!");
}

void loop(){
}
