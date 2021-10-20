# DFRobot_MultiGasSensor
- [中文版](./README_CN.md)

## DFRobot_MultiGasSensor Library for Arduino
---------------------------------------------------------
气体传感器广泛应用在气体研究，环境检测，生产安全监测，溶解气体分析，污染源/排放口规律研究，有毒有害，可燃气体检测报警，化验室或现场简单气体分析测试等方面，这款气体传感器更是集成了多种气体探头的一个多气体传感器，可以适用于各种各样的应用场景
![正反面svg效果图](./resources/images/DFR0784svg1.png)

## Product Link（链接到中文商城）
SKU：DFR0784

## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

## Summary

这是一个用于复杂环境中检测多种气体的传感器，支持O2 CO H2S 
NO2 O3 CL2 NH3 H2 HCL SO2 HF PH3等气体。只需要硬件切换对应
的探头就可以了。同时支持气体高阈值或者低阈值报警功能

## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_MultiGasSensor.
Download the zip file first to use this library and uncompress it to a folder named DFRobot_MultiGasSensor.

## Methods

```C++
  /**
   * @fn begin
   * @brief 父类初始化，在子类函数中会进行IIC或者UART初始化
   * @return bool类型，表示初始化是否成功
   * @retval True 成功
   * @retval False 失败
   */
  virtual bool begin(void) = 0;

  /**
   * @fn changeAcquireMode
   * @brief 改变获取传感器数据的方式
   * @param mode 模式选择
   * @n     INITIATIVE 传感器主动上报数据
   * @n     PASSIVITY 需要主控这边向传感器请求数据
   * @return bool类型，表示设置是否成功
   * @retval True 成功
   * @retval False 失败
   */
  bool changeAcquireMode(eMethod_t mode);

  /**
   * @fn readGasConcentrationPPM
   * @brief 获取传感器浓度，单位是PPM
   * @return float类型，表示返回气体浓度，如果数据传输正常，那么返回气体浓度，否则，返回0.0
   */
  float readGasConcentrationPPM(void);

  /**
   * @fn queryGasType
   * @brief 查询气体类型
   * @return String类型，表示返回气体类型字符串
   */
  String queryGasType(void);

  /**
   * @fn setThresholdAlarm
   * @brief 设置传感器报警的阈值
   * @param switchof 是否打开阈值报警开关
   * @n            ON 打开     
   * @n           OFF 关闭
   * @param threshold 开始报警的阈值大小
   * @param alamethod 设置传感器高阈值或者低阈值报警
   * @param gasType   气体类型
   * @return bool类型，表示设置是否成功
   * @retval True 成功
   * @retval False 失败
   */
  bool setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, String gasType);

  /**
   * @fn readTempC
   * @brief 获取传感器的板载温度
   * @return float类型，表示返回当前板子的温度
   */
  float readTempC(void);

  /**
   * @fn setTempCompensation
   * @brief 设置是否开启温度补偿，传感器在不同温度下的输出值会有差别，所以为
   * @n     了获取到的气体浓度更精确，在计算气体浓度的时候需要增加温度补偿
   * @param tempswitch 是否打开温度补偿
   * @n             ON 打开温度补偿
   * @n            OFF 关闭温度补偿
   */
  void setTempCompensation(eSwitch_t tempswitch);

  /**
   * @fn readVolatageData
   * @brief 获取传感器气体浓度以原始电压输出，不同于直接读取传感器寄存器，这
   * @n     个函数主要用来检验读取的气体浓度是否准确
   * @param vopin 用来接收传感器探头原始电压输出的引脚
   * @return float类型，表示返回传感器气体浓度的原始电压输出
   */
  float readVolatageData(uint8_t vopin);

  /**
   * @fn pack
   * @brief 将协议的数据进行打包以便于传输
   * @param pBuf 等待打包的数据
   * @param pBuf 数据包长度  
   * @return sProtocol_t类型，表示返回打包好的数据
   */
  sProtocol_t pack(uint8_t *pBuf, uint8_t pBuf);

  /**
   * @fn getSensorVoltage
   * @brief 获取传感器探头输出的电压（用来计算此时的气体浓度）
   * @return float类型，表示返回电压值
   */
  float getSensorVoltage(void);

  /**
   * @fn dataIsAvailable
   * @brief 在主动模式下调用此函数，用以判断数据线上有没有数据
   * @return bool类型，表示数据是否传感器有数据上传过来
   * @retval True 有数据上传
   * @retval False 无数据上传
   */
  virtual bool dataIsAvailable(void) = 0;

  /**
   * @fn changeI2cAddrGroup
   * @brief 改变I2C地址组
   * @param group 地址组选择
   * @return int类型，表示返回初始化的状态
   * @retval bool类型
   * @retval True 修改成功
   * @retval False 修改失败
   */
  bool changeI2cAddrGroup(uint8_t group);
```
## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266|      √       |              |             | 
Mega2560  |      √       |             |            | 
Arduino uno |       √      |             |            | 
Leonardo  |      √       |              |             | 
Micro：bit  |      √       |              |             | 
M0  |      √       |              |             | 

## History

- 02,04, 2021 - Version 2.0 released.


## Credits

Written by PengKaixing(kaixing.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))