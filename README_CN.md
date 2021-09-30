# DFRobot_MultiGasSensor
DFRobot's MultiGasSensor

## DFRobot_MultiGasSensor Library for Arduino
---------------------------------------------------------
这是一个用于复杂环境中检测多种气体的传感器，支持O2 CO H2S 
NO2 O3 CL2 NH3 H2 HCL SO2 HF PH3等气体。只需要硬件切换对应
的探头就可以了。同时支持气体高阈值或者低阈值报警功能
## Product Link（链接到英文商城）
SKU：SEN0502

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
    bool changeAcquireMode(eMethod_t mode);

/*!
 *  @brief 获取传感器获取的气体浓度或者气体的类型，单位是PPM
 *  @return 如果数据传输正常，那么返回气体浓度
 *          否则，返回0.0
 */
    float readGasConcentrationPPM(void);

/*!
 *  @brief 获取传感器获取气体的类型
 *  @param 无
 *  @return 气体类型
            O2 = 0x00,
            CO = 0x01,
            H2S = 0x02,
            NO2 = 0x03,
            O3 = 0x04,
            CL2 = 0x05,
            NH3 = 0x06,
            H2 = 0x07,
            HCL = 0X08,
            SO2 = 0X09,
            HF = 0x0A,
            _PH3 = 0x0B
          通信失败返回0xff
*/
    String queryGasType(void);
/*!
 *  @brief 设置传感器报警的阈值
 *  @param switchof    ：设置是否打开报警
           ON          ：打开报警功能
           OFF         ：关闭报警功能
           threshold   ：设置报警的阈值         
 *  @return status  ： init status
 *          true is ： init success
 *          false is： init error
 */
    bool setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod, String gasType);

/*!
 *  @brief 获取传感器的板载温度
 *  @param 无
 *  @return 以float类型返回当前板子的温度
 */
    float readTempC(void);

/*!
 *  @brief 设置是否开启温度补偿，传感器在不同温度下的输出值会有差别，所以
           为了获取到的气体浓度更精确，在计算气体浓度的时候需要增加温度补偿
 *  @param tempswitch：
           ON          ：打开温度补偿
           OFF         ：关闭温度补偿
 *  @return 无
 */
    void setTempCompensation(eSwitch_t tempswitch);

/*!
 *  @brief 获取传感器气体浓度以原始电压输出，不同于直接读取传感器寄存器，这
           个函数主要用来检验读取的气体浓度是否准确
 *  @param  vopin：用来接收传感器探头原始电压输出的引脚
 *  @return 传感器气体浓度的原始电压输出
 */
    float readVolatageData(uint8_t vopin);

/*!
 *  @brief 将协议的数据进行打包以便于传输
 *  @param  等待打包的数据
 *          pBuf：传入的数据包的指针
 *  @param  数据包长度  
 *          len：长度数值
 *  @return 打包好的数据
 */
    sProtocol_t pack(uint8_t *pBuf, uint8_t len);

/*!
 *  @brief 获取传感器探头输出的电压（用来计算此时的气体浓度）
 *  @param  无
 *  @return 电压值
 */
    float getSensorVoltage(void);

/*!
 *  @brief 在主动模式下调用此函数，用以判断数据线上有没有数据
 *  @param  无
 *  @return status  ： status
 *          true is ： success is Available
 *          false is： error is unavailable
 */
    virtual bool dataIsAvailable(void) = 0;

/*!
 *  @brief 软件改变iic地址
 *  @param  group iic地址组
 *  @return true is ： 修改成功
 *          false is： 修改失败
 */
    bool changeI2cAddrGroup(uint8_t group);
  protected:
/*!
 *  @brief 向传感器的指定寄存器写入数据
 *  @param 需要写入的寄存器地址
 *         Reg ：寄存器地址
 *  @param 等待写入寄存器的数据
 *         Data：数据指针
 *  @param 等待写入的数据的长度
 *         len ：数据长度
 *  @return 没有返回值
 */
    virtual void writeData(uint8_t Reg, void *Data, uint8_t len) = 0;

/*!
 *  @brief 从指定传感器中获取指定长度的数据
 *  @param 需要读取的寄存器地址
 *         Reg ：寄存器地址
 *  @param 等待读取寄存器的数据存入的位置
 *         Data：数据指针
 *  @param 等待读取的数据的长度
 *         len ：数据长度
 *  @return 真实读取到数据长度
 */
    virtual int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len) = 0;
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