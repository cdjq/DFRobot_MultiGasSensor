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

#define communication_IIC              1
#define communication_UART             0

// Open this macro to see the program running in detail
#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] 0x"); Serial.println(__VA_ARGS__,HEX);}
#else
#define DBG(...)
#endif

typedef struct
{
  uint8_t head;
  uint8_t addr;
  uint8_t data[6];
  uint8_t check;
} sProtocol_t;

/*
  DFR0784 主动上报协议
   * -----------------------------------------------------------------------------------------------------------
   * | byte0  | byte1 |    byte2   |    byte3    |  byte4  |  byte5   |  byte6   |  byte7  | byte8 |
   * -----------------------------------------------------------------------------------------------------------
   * | 起始位 | 命令  |  浓度高字节 |  浓度低字节 | 气体类型|温度高字节|温度低字节 |  保留   | 校验  |
   * -----------------------------------------------------------------------------------------------------------
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
  DFRobot_GAS(){};
  ~DFRobot_GAS(){};

  //改变获取气体方式
  typedef enum
  {
    INITIATIVE = 0x03,
    PASSIVITY = 0x04
  } eMethod_t;

  //气体类型
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

  //是否打开ALA报警功能
  typedef enum
  {
    ON = 0x01,
    OFF = 0x00
  } eSwitch_t;

  typedef enum
  {
    TWENTY_FIVE = 0x01,
    FIFTH = 0x02,
    SEVENTY_FIVE = 0x03
  } ePWM_t;

  typedef enum
  {
    LOW_THRESHOLD_ALA = 0x00,
    HIGH_THRESHOLD_ALA = 0x01
  } eALA_t;
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
  float readGasConcentrationPPM();

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
  String queryGasType();
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
 *  @param Reg ：需要写入的寄存器地址
 *         Data：等待写入寄存器的数据
 *         len ：等待写入的数据的长度
 *  @return 没有返回值
 */
  virtual void writeData(uint8_t Reg, void *Data, uint8_t len) = 0;

/*!
 *  @brief 从指定传感器中获取指定长度的数据
 *  @param INITIATIVE：传感器主动上报
 *         PASSIVITY ：主控发送请求，传感器才能上报数据
 *  @return status  ：init status
 *          true is ：init success
 *          false is：init error
 */
  virtual int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len) = 0;

private:
  bool _tempswitch;
};

class DFRobot_GAS_I2C : public DFRobot_GAS{
  public:
    DFRobot_GAS_I2C(TwoWire *pWire=&Wire,uint8_t addr=0x74);
    ~DFRobot_GAS_I2C(){};
    bool begin(void);
    float setI2cAddr(uint8_t addr);
    bool dataIsAvailable();
  protected:
    void writeData(uint8_t Reg ,void *Data ,uint8_t len);
    int16_t readData(uint8_t Reg ,uint8_t *Data ,uint8_t len);
  private:
    TwoWire* _pWire;
    uint8_t _I2C_addr;
};

#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
class DFRobot_GAS_SoftWareUart : public DFRobot_GAS{
  public:
    DFRobot_GAS_SoftWareUart(SoftwareSerial *psoftUart);
    ~DFRobot_GAS_SoftWareUart(){};
    bool begin(void);
    bool dataIsAvailable();
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
    ~DFRobot_GAS_HardWareUart(){};
    bool begin(void);
  protected:
    bool dataIsAvailable();
    void writeData(uint8_t Reg, void *Data, uint8_t len);
    int16_t readData(uint8_t Reg, uint8_t *Data, uint8_t len);
  private:
    HardwareSerial *_pharduart;
};
#endif

#endif