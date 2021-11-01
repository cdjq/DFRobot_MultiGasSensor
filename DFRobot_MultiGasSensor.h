/*!
  * @file  DFRobot_MultiGasSensor.h
  * @brief 这是一个可以检测空气中气体浓度传感器的库的头文件
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V2.0
  * @date        2021-09-26
  * @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
*/
#ifndef __DFRobot_GAS_H__
#define __DFRobot_GAS_H__

#include "Arduino.h"
#include <Wire.h>

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
#include "SoftwareSerial.h"
#else
#include "HardwareSerial.h"
#endif

#define CMD_CHANGE_GET_METHOD          0X78
#define CMD_GET_GAS_CONCENTRATION      0X86
#define CMD_GET_TEMP                   0X87
#define CMD_GET_ALL_DTTA               0X88
#define CMD_SET_THRESHOLD_ALARMS       0X89
#define CMD_IIC_AVAILABLE              0X90
#define CMD_SENSOR_VOLTAGE             0X91
#define CMD_CHANGE_IIC_ADDR            0X92

// Open this macro to see the program running in detail
#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] 0x"); Serial.println(__VA_ARGS__,HEX);}
#else
#define DBG(...)
#endif

/**
 * @struct sProtocol_t
 * @brief 通信用的数据协议包
 */
typedef struct
{
  uint8_t head;
  uint8_t addr;
  uint8_t data[6];
  uint8_t check;
} sProtocol_t;

/**
 * @struct sAllData_t
 * @brief 获取全部数据时用到的结构体
 * @note 这里从数据手册上抄写关于这个寄存器的描述
 * @n --------------------------------------------------------------------------------------------------------
 * @n |  byte0   | byte1 |    byte2     |    byte3      |  byte4   |  byte5   |  byte6   |   byte7   |  byte8   
 * @n --------------------------------------------------------------------------------------------------------
 * @n |  协议头  | 命令  | 气体浓度高8位 | 气体浓度低8位 | 气体类型 | 小数位数 | 温度高8位 | 温度低8位 |  CRC
 * @n --------------------------------------------------------------------------------------------------------
 */
typedef struct
{
  uint8_t head;
  uint8_t cmd;
  uint8_t gasconcentration_h;
  uint8_t gasconcentration_l;
  uint8_t gastype;
  uint8_t gasconcentration_decimals;
  uint8_t temp_h;
  uint8_t temp_l;
  uint8_t check;
} sAllData_t;
extern sAllData_t AllData;

/**
 * @struct sAllDataAnalysis_t
 * @brief 解析完成的数据
 */
typedef struct
{
  float gasconcentration;
  String gastype;
  float temp;
} sAllDataAnalysis_t;
extern sAllDataAnalysis_t AllDataAnalysis;

class DFRobot_GAS
{
public:
  /**
   * @enum eMethod_t
   * @brief 传感器上传数据类型
   */
  typedef enum
  {
    INITIATIVE = 0x03,
    PASSIVITY = 0x04
  } eMethod_t;

  /**
   * @enum eType_t
   * @brief 气体类型
   */
  typedef enum
  {
    O2 = 0x05,
    CO = 0x04,
    H2S = 0x03,
    NO2 = 0x2C,
    O3 = 0x2A,
    CL2 = 0x31,
    NH3 = 0x02,
    H2 = 0x06,
    HCL = 0X2E,
    SO2 = 0X2B,
    HF = 0x33,
    _PH3 = 0x45
  } eType_t;

  /**
   * @enum eSwitch_t
   * @brief 是否打开ALA报警功能
   */
  typedef enum
  {
    ON = 0x01,
    OFF = 0x00
  } eSwitch_t;

  /**
   * @enum eSwitch_t
   * @brief 高低ALA报警功能
   */
  typedef enum
  {
    LOW_THRESHOLD_ALA = 0x00,
    HIGH_THRESHOLD_ALA = 0x01
  } eALA_t;

  DFRobot_GAS(void){};
  ~DFRobot_GAS(void){};

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
   * @param len 数据包长度  
   * @return sProtocol_t类型，表示返回打包好的数据
   */
  sProtocol_t pack(uint8_t *pBuf, uint8_t len);

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

protected:
  /**
   * @fn writeData
   * @brief 向传感器的指定寄存器写入数据
   * @param Reg 需要写入的寄存器地址
   * @param Data 等待写入寄存器的数据
   * @param len 等待写入的数据的长度
   */
  virtual void writeData(uint8_t Reg, void *Data, uint8_t len) = 0;

  /**
   * @fn readData
   * @brief 从指定传感器中获取指定长度的数据
   * @param Reg 需要读取的寄存器地址
   * @param Data 等待读取寄存器的数据存入的位置
   * @param len 等待读取的数据的长度
   */
  virtual int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len) = 0;

private:
  bool _tempswitch;
};

class DFRobot_GAS_I2C : public DFRobot_GAS
{
  public:
    DFRobot_GAS_I2C(TwoWire *pWire=&Wire,uint8_t addr=0x74);
    ~DFRobot_GAS_I2C(void){};
    bool begin(void);
    float setI2cAddr(uint8_t addr);
    bool dataIsAvailable(void);
  protected:
    void writeData(uint8_t Reg ,void *Data ,uint8_t len);
    int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  private:
    TwoWire* _pWire;
    uint8_t _I2C_addr;
};

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
class DFRobot_GAS_SoftWareUart : public DFRobot_GAS
{
  public:
    DFRobot_GAS_SoftWareUart(SoftwareSerial *psoftUart);
    ~DFRobot_GAS_SoftWareUart(void){};
    bool begin(void);
    bool dataIsAvailable(void);
  protected:
    void writeData(uint8_t Reg ,void *Data ,uint8_t len);
    int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  private:
    SoftwareSerial *_psoftUart;
}; 
#else
class DFRobot_GAS_HardWareUart : public DFRobot_GAS
{
  public:
    DFRobot_GAS_HardWareUart(HardwareSerial *phardUart);
    ~DFRobot_GAS_HardWareUart(void){};
    bool begin(void);
  protected:
    bool dataIsAvailable(void);
    void writeData(uint8_t Reg, void *Data, uint8_t len);
    int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len);
  private:
    HardwareSerial *_pharduart;
};

#endif
#endif