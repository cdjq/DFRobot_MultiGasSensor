/*!
  * @file  readVolatageData.ino
  * @brief 获取当前环境对应气体浓度以电压模拟输出
  * @n 通信方式选择，拨码开关SEL：0：IIC,1：UART
  * @n 组序号         组内地址
  * @n A0 A1拨码电平 00    01    10    11
  * @n 1            0x60  0x61  0x62  0x63
  * @n 2            0x64  0x65  0x66  0x67
  * @n 3            0x68  0x69  0x6A  0x6B
  * @n 4            0x6C  0x6D  0x6E  0x6F
  * @n 5            0x70  0x71  0x72  0x73
  * @n 6（默认地址组） 0x74  0x75  0x76  0x77（默认地址）
  * @n 7            0x78  0x79  0x7A  0x7B
  * @n 8            0x7C  0x7D  0x7E  0x7F
  * @n i2c 地址选择，默认i2c地址为0x77，A1、A0组合成4种IIC地址
  * @n             | A0 | A1 |
  * @n             | 0  | 0  |    0x74
  * @n             | 0  | 1  |    0x75
  * @n             | 1  | 0  |    0x76
  * @n             | 1  | 1  |    0x77   default i2c address  
  * @n 实验现象： 在串口打印可以看到此时传感器的电压
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V1.0
  * @date        2021-03-28
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
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
}

void loop() {
/**
  在readVolatageData()中填入实际与传感器的A0引脚连接的模拟引脚
  在串口可以看到此时传感器输出的电压值（模拟值已转换成电压值）
  每次延时1s打印
*/
  Serial.print("Sensor output voltage is: ");
  Serial.print(gas.getSensorVoltage());
  Serial.println(" V");
  Serial.println();
  delay(1000);
}