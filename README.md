# DFRobot_MultiGasSensor
DFRobot's MultiGasSensor

## DFRobot_MultiGasSensor Library for Arduino
---------------------------------------------------------



## Table of Contents

* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

<snippet>
<content>

## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_MultiGasSensor.
Download the zip file first to use this library and uncompress it to a folder named DFRobot_MultiGasSensor.

## Methods

```C++
class DFRobot_MultiGasSensor
/*!
 *  @brief 父类初始化，在子类函数中会进行IIC或者UART初始化
 *
 *  @return status  ： init status
 *          true is ： init success
 *          false is： init error
 */
    virtual bool begin(void) = 0;
  
/*!
 *  @brief 改变传感器采集到气体以后数据上报到主控的方式
 *  @param INITIATIVE：传感器主动上报
 *         PASSIVITY ：主控发送请求，传感器才能上报数据
 *  @return status
 *          true is ： change success
 *          false is： change fail
 */
    bool changeAcquireMode(uint8_t mode);
  
/*!
 *  @brief 获取传感器获取的气体浓度或者气体的类型
 *  @param gastype    ：此时的传感器的类型
           O2 ：氧气
           CO ：一氧化碳
           H2S：硫化氢
           NO2：二氧化氮
           O3 ：臭氧
           CL2：氯气
           NH3：氨气
           H2 ：氢气
           HF ：氟化氢
           PH3：磷化氢
 *  @return 如果数据传输正常，那么返回气体浓度
 *          否则，返回0.0
 */
    float readGasConcentration(uint8_t gastype);
  
/*!
 *  @brief 获取传感器获取气体的类型
 *  @param 无
 *  @return 气体类型
            O2 ：0x00 
            CO ：0x01
            H2S：0x02
            NO2：0x04
            O3 ：0x05
            CL2：0x06
            NH3：0x07
            H2 ：0x08
            HF ：0x0A
            PH3：0x0B
            通信失败返回0xff
 */
    uint8_t readGasType();
/*!
 *  @brief 设置传感器报警的阈值
 *  @param switchof    ：设置是否打开报警
           ON          ：打开报警功能
           OFF         ：关闭报警功能
           threshold   ：设置报警的阈值
           returntype ：
           GASCON     :气体浓度
           GASKIND    :气体种类
 *  @return status  ： init status
 *          true is ： init success
 *          false is： init error
 */
    bool setThresholdAlarm(uint8_t switchof,uint16_t threshold=200, eALA_t alamethod);
  
/*!
 *  @brief 获取传感器的板载温度
 *  @param 无
 *  @return 以float类型返回当前板子的温度
 */
    float readTemp(void);
   
/*!
 *  @brief 设置是否开启温度补偿，传感器在不同温度下的输出值会有差别，所以
           为了获取到的气体浓度更精确，在计算气体浓度的时候需要增加温度补偿
 *  @param tempswitch：
           ON          ：打开温度补偿
           OFF         ：关闭温度补偿
 *  @return 无
 */
    void setTempCompensation(bool tempswitch);

/*!
 *  @brief 获取传感器气体浓度以原始电压输出，不同于直接读取传感器寄存器，这
           个函数主要用来检验读取的气体浓度是否准确
 *  @param  vopin：用来接收传感器探头原始电压输出的引脚
 *  @return 传感器气体浓度的原始电压输出
 */
    float readVolatageData(uint8_t vopin);

/*!
 *  @brief 将协议的数据进行打包以便于传输
 *  @param  pBuf：等待打包的数据
 *          len：数据包长度  
 *  @return 打包好的数据
 */
  sProtocol_t pack(uint8_t *pBuf, uint8_t len);

/*!
 *  @brief 在不同的浓度特征点，输出占空比不同的PWM信号
 *  @param  无
 *  @return 无
 */
  void setCustomizeIO(ePWM_t duty);

/*!
 *  @brief 在主动模式下调用此函数，用以判断数据线上有没有数据
 *  @param  无
 *  @return status  ： status
 *          true is ： success is Available
 *          false is： error is unavailable
 */
  bool dataIsAvailable(void);
```
## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266|      √       |              |             | 
Mega2560  |      √       |             |            | 
Arduino uno |       √      |             |            | 
Leonardo  |      √       |              |             | 




## History

- 02,04, 2021 - Version 0.2 released.


## Credits

Written by PengKaixing(kaixing.peng@dfrobot.com), 2021. (Welcome to our [website](https://www.dfrobot.com/))