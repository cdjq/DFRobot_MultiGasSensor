/*!
  * @file  setThresholdAlarm.ino
  * @brief 设置传感器的阈值报警
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
  * @n 实验现象： 当传感器获取到的数据值超过设置的阈值之后，传感器的ALA引脚将输出
                  高电平
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V1.0
  * @date        2021-03-28
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/dfrobot/DFRobot_MultiGasSensor
*/
#include "DFRobot_MultiGasSensor.h"

//默认打开，此时使用IIC通信，屏蔽之后使用软串口通信
#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x77
  DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
#else
#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
/**
  UNO:pin_2-----RX
      pin_3-----TX
*/
  SoftwareSerial mySerial(2, 3);
  DFRobot_GAS_SoftWareUart gas(&mySerial);
#else
/**
  ESP32:IO16-----RX
        IO17-----TX
*/
  DFRobot_GAS_HardWareUart gas(&Serial2); //ESP32HardwareSerial
#endif
#endif

/**
  设置与报警引脚连接的主控引脚
*/
#define ALA_pin 3

void setup() {
/**
  串口初始化，用作查看打印输出
*/
  Serial.begin(115200);
  
/**
  传感器初始化，用作初始化串口或者初始化IIC，由此时使用的通信方式来决定
*/
  while(!gas.begin())
  {
    Serial.println("NO Deivces !");
    delay(1000);
  }
/**
  获取数据模式为：主控需要向传感器请求
*/
  while (!gas.changeAcquireMode(gas.PASSIVITY))
  {
    delay(1000);
  }
  Serial.println("change acquire mode success!");


/**
  设置传感器报警的阈值
*/
  while (!gas.setThresholdAlarm(/*是否打开阈值报警*/ gas.ON, 200, gas.LOW_THRESHOLD_ALA,gas.queryGasType())
  {
    Serial.println("Failed to open alarm!");
    delay(1000);
  }
  pinMode(ALA_pin,INPUT);
}

void loop() {
/**
  循环获取环境气体浓度,到阈值则报警
*/
  gas.readGasConcentrationPPM();
  if (digitalRead(ALA_pin) == 1)
  {
    Serial.println("warning!!!");
  }
  else
  {
    Serial.println("nolmal!!!");
  }
  delay(1000);
}