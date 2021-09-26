/*!
  * @file  DFRobot_MultiGasSensor.h
  * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @licence     The MIT License (MIT)
  * @author      PengKaixing(kaixing.peng@dfrobot.com)
  * @version     V2.0
  * @date        2021-09-26
  * @get         from https://www.dfrobot.com
  * @url         https://github.com/DFRobot/DFRobot_MultiGasSensor
*/
#include "DFRobot_MultiGasSensor.h"

sAllData_t AllData;
sAllDataAnalysis_t AllDataAnalysis;
float _temp;

static uint8_t FucCheckSum(uint8_t* i,uint8_t ln)
{
  uint8_t j,tempq=0;
  i+=1;
  for(j=0;j<(ln-2);j++)
  {
    tempq+=*i;i++;
  }
  tempq=(~tempq)+1;
  return(tempq);
}

static void analysisAllData(void)
{
  float Con;
  if (AllData.check == FucCheckSum((uint8_t *)&AllData, 8))
  {
    switch(AllData.gasconcentration_decimals){
      case 0:
        AllDataAnalysis.gasconcentration = (AllData.gasconcentration_h << 8) + AllData.gasconcentration_l;
        break;
      case 1:
        AllDataAnalysis.gasconcentration = 0.1 * ((AllData.gasconcentration_h << 8) + AllData.gasconcentration_l);
        break;
      case 2:
        AllDataAnalysis.gasconcentration = 0.01 * ((AllData.gasconcentration_h << 8) + AllData.gasconcentration_l);
        break;
      default:
        break;
    }
    Con = AllDataAnalysis.gasconcentration;
    switch (AllData.gastype){
      case DFRobot_GAS::O2:
        break;
      case DFRobot_GAS::CO:
        if (((_temp) > -20) && ((_temp) < 20))
          Con = (Con / (0.005 * (_temp) + 0.9));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = (Con / (0.005 * (_temp) + 0.9) - (0.3 * (_temp) - 6));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::H2S:
        if (((_temp) > -20) && ((_temp) < 20))
          Con = (Con / (0.006 * (_temp) + 0.92));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = (Con / (0.006 * (_temp) + 0.92) - (0.015 * (_temp) + 2.4));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::NO2:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ((Con / (0.005 * (_temp) + 0.9) - (-0.0025 * (_temp))));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / (0.005 * (_temp) + 0.9) - (0.005 * (_temp) + 0.005)));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / (0.005 * (_temp) + 0.9) - (0.0025 * (_temp) + 0.1)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::O3:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ( (Con / (0.015 * (_temp) + 1.1) - 0.05));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / 1.1 - (0.01 * (_temp))));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / 1.1 - (-0.05 * (_temp) + 0.3)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::CL2:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ( (Con / (0.015 * (_temp) + 1.1) - (-0.0025 * (_temp))));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / 1.1 - 0.005 * (_temp)));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / 1.1 - (0.06 * (_temp) - 0.12)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::NH3:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = (Con / (0.08 * (_temp) + 3.98) - (-0.005 * (_temp) + 0.3));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = (Con / (0.08 * (_temp) + 3.98) - (-0.005 * (_temp) + 0.3));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = (Con / (0.004 * (_temp) + 1.08) - (-0.1 * (_temp) + 2));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::H2:
        if (((_temp) > -20) && ((_temp) < 40))
          Con = (Con / (0.74 * (_temp) + 0.007) - 5);
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::HF:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ( ((Con / 1) - (-0.0025 * (_temp))));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / 1 + 0.1));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / 1 - (0.0375 * (_temp) - 0.85)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::_PH3:
        if (((_temp) > -20) && ((_temp) < 40))
          Con = ( (Con / (0.005 * (_temp) + 0.9)));
        break;
      case DFRobot_GAS::HCL:
        break;
      case DFRobot_GAS::SO2:
        break;
      default:
        break;
    }
    if (Con>=0)
      AllDataAnalysis.gasconcentration = Con;
    else
      AllDataAnalysis.gasconcentration = 0;
    switch (AllData.gastype){
      case 0x05:
        AllDataAnalysis.gastype = "O2";
        break;
      case 0x04:
        AllDataAnalysis.gastype = "CO";
        break;
      case 0x03:
        AllDataAnalysis.gastype = "H2S";
        break;
      case 0x2C:
        AllDataAnalysis.gastype = "NO2";
        break;
      case 0x2A:
        AllDataAnalysis.gastype = "O3";
        break;
      case 0x31:
        AllDataAnalysis.gastype = "CL2";
        break;
      case 0x02:
        AllDataAnalysis.gastype = "NH3";
        break;
      case 0x06:
        AllDataAnalysis.gastype = "H2";
        break;
      case 0x2E:
        AllDataAnalysis.gastype = "HCL";
        break;
      case 0x2B:
        AllDataAnalysis.gastype = "SO2";
        break;
      case 0x33:
        AllDataAnalysis.gastype = "HF";
        break;
      case 0x45:
        AllDataAnalysis.gastype = "PH3";
        break;
      default:
        AllDataAnalysis.gastype = "";
        break;
    }
    uint16_t temp_ADC = (AllData.temp_h << 8) + AllData.temp_l;
    float Vpd3 = 3 * (float)temp_ADC / 1024;
    float Rth = Vpd3 * 10000 / (3 - Vpd3);
    AllDataAnalysis.temp = 1 / (1 / (273.15 + 25) + 1 / 3380.13 * log(Rth / 10000)) - 273.15;
  }
}

sProtocol_t DFRobot_GAS::pack(uint8_t *pBuf, uint8_t len)
{ 
  sProtocol_t _protocol;
  _protocol.head = 0xff;
  _protocol.addr = 0x01;
  memcpy(_protocol.data, pBuf, len);
  _protocol.check = FucCheckSum((uint8_t *)&_protocol, 8);
  return _protocol;
}

bool DFRobot_GAS::changeAcquireMode(eMethod_t mode)
{
  uint8_t buf[6]={0};
  uint8_t recvbuf[9]={0};
  buf[0] = CMD_CHANGE_GET_METHOD;
  buf[1] = mode;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  delay(100);
  readData(0,recvbuf,9);
  if(recvbuf[2]==1){
    return true;
  }else{
    return false;
  }
}

float DFRobot_GAS::readGasConcentrationPPM()
{
  uint8_t buf[6] = {0};
  uint8_t recvbuf[9] = {0};
  uint8_t gastype;
  uint8_t decimal_digits;
  buf[0] = CMD_GET_GAS_CONCENTRATION;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  delay(100);
  readData(0, recvbuf, 9);
  float Con=0.0;
  if(FucCheckSum(recvbuf,8) == recvbuf[8]){
    Con=((recvbuf[2]<<8)+recvbuf[3])*1.0;
    gastype = recvbuf[4];
    decimal_digits = recvbuf[5];
    switch(decimal_digits){
      case 1:
        Con *= 0.1;
        break;
      case 2:
        Con *= 0.01;
        break;
      default:
        break;
    }
    if (_tempswitch == DFRobot_GAS::OFF)
      return Con;
    switch (gastype){
      case DFRobot_GAS::O2:
        break;
      case DFRobot_GAS::CO:
        if (((_temp) > -20) && ((_temp) < 20))
          Con = (Con / (0.005 * (_temp) + 0.9));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = (Con / (0.005 * (_temp) + 0.9) - (0.3 * (_temp) - 6));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::H2S:
        if (((_temp) > -20) && ((_temp) < 20))
          Con = (Con / (0.006 * (_temp) + 0.92));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = (Con / (0.006 * (_temp) + 0.92) - (0.015 * (_temp) + 2.4));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::NO2:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ((Con / (0.005 * (_temp) + 0.9) - (-0.0025 * (_temp))));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / (0.005 * (_temp) + 0.9) - (0.005 * (_temp) + 0.005)));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / (0.005 * (_temp) + 0.9) - (0.0025 * (_temp) + 0.1)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::O3:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ( (Con / (0.015 * (_temp) + 1.1) - 0.05));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / 1.1 - (0.01 * (_temp))));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / 1.1 - (-0.05 * (_temp) + 0.3)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::CL2:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ( (Con / (0.015 * (_temp) + 1.1) - (-0.0025 * (_temp))));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / 1.1 - 0.005 * (_temp)));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / 1.1 - (0.06 * (_temp) - 0.12)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::NH3:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = (Con / (0.08 * (_temp) + 3.98) - (-0.005 * (_temp) + 0.3));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = (Con / (0.08 * (_temp) + 3.98) - (-0.005 * (_temp) + 0.3));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = (Con / (0.004 * (_temp) + 1.08) - (-0.1 * (_temp) + 2));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::H2:
        if (((_temp) > -20) && ((_temp) < 40))
          Con = (Con / (0.74 * (_temp) + 0.007) - 5);
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::HF:
        if (((_temp) > -20) && ((_temp) < 0))
          Con = ( ((Con / 1) - (-0.0025 * (_temp))));
        else if (((_temp) > 0) && ((_temp) < 20))
          Con = ( (Con / 1 + 0.1));
        else if (((_temp) > 20) && ((_temp) < 40))
          Con = ( (Con / 1 - (0.0375 * (_temp) - 0.85)));
        else
          Con = 0.0;
        break;
      case DFRobot_GAS::_PH3:
        if (((_temp) > -20) && ((_temp) < 40))
          Con = ( (Con / (0.005 * (_temp) + 0.9)));
        break;
      case DFRobot_GAS::HCL:
        break;
      case DFRobot_GAS::SO2:
        break;
      default:
        break;
    }
  }else{
    Con = 0.0;
  }
  if (Con<0)
    return 0;
  else
    return Con;
}

String DFRobot_GAS::queryGasType()
{
  uint8_t buf[6] = {0};
  uint8_t recvbuf[9] = {0};
  buf[0] = CMD_GET_GAS_CONCENTRATION;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  delay(100);
  readData(0, recvbuf, 9);
  if(FucCheckSum(recvbuf,8) == recvbuf[8]){
    switch(recvbuf[4]){
      case 0x05:
        return "O2";
        break;
      case 0x04:
        return "CO";
        break;
      case 0x03:
        return "H2S";
        break;
      case 0x2C:
        return "NO2";
        break;
      case 0x2A:
        return "O3";
        break;
      case 0x31:
        return "CL2";
        break;
      case 0x02:
        return "NH3";
        break;
      case 0x06:
        return "H2";
        break;
      case 0x2E:
        return "HCL";
        break;
      case 0x2B:
        return "SO2";
        break;
      case 0x33:
        return "HF";
        break;
      case 0x45:
        return "PH3";
        break;
      default:
        return "";
        break;
    }
  }else{
    return "NO GAS";
  }
}

bool DFRobot_GAS::setThresholdAlarm(eSwitch_t switchof, uint16_t threshold, eALA_t alamethod,String gasType)
{
  if (gasType == "O2")
    threshold *= 10;
  else if (gasType == "NO2")
    threshold *= 10;
  else if (gasType == "O3")
    threshold *= 10;
  else if (gasType == "CL2")
    threshold *= 10;
  else if (gasType == "HCL")
    threshold *= 10;
  else if (gasType == "SO2")
    threshold *= 10;
  else if (gasType == "HF")
    threshold *= 10;
  else if (gasType == "PH3")
    threshold *= 10;
  uint8_t buf[6] = {0};
  uint8_t recvbuf[9] = {0};
  buf[0] = CMD_SET_THRESHOLD_ALARMS;
  buf[1] = switchof;
  buf[2] = (int)threshold >> 8;
  buf[3] = (int)threshold;
  buf[4] = alamethod;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  delay(100);
  readData(0, recvbuf, 9);
  if (recvbuf[8]!=FucCheckSum(recvbuf,8))
    return false;
  if(recvbuf[2]==1)
    return true;
  else 
    return false;
}

float DFRobot_GAS::readTempC(void)
{
  uint8_t buf[6] = {0};
  uint8_t recvbuf[9] = {0};
  buf[0] = CMD_GET_TEMP;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  delay(100);
  readData(0, recvbuf, 9);
  if (recvbuf[8] != FucCheckSum(recvbuf, 8))
    return 0.0;
  uint16_t temp_ADC = (recvbuf[2] << 8) + recvbuf[3];
  float Vpd3=3*(float)temp_ADC/1024;
  float Rth = Vpd3*10000/(3-Vpd3);
  float Tbeta = 1/(1/(273.15+25)+1/3380.13*log(Rth/10000))-273.15;
  return Tbeta;
}

void DFRobot_GAS::setTempCompensation(eSwitch_t tempswitch)
{
  _tempswitch = tempswitch;
  _temp=readTempC();
}

float DFRobot_GAS::getSensorVoltage(void)
{
  uint8_t buf[6] = {0};
  uint8_t recvbuf[9] = {0};
  buf[0] = CMD_SENSOR_VOLTAGE;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  delay(100);
  readData(0, recvbuf, 9);
  if (recvbuf[8] != FucCheckSum(recvbuf, 8))
    return 0.0;
  else
    return ((uint16_t )((recvbuf[2] << 8) + recvbuf[3])*3.0/1024*2);
}

bool DFRobot_GAS::changeI2cAddrGroup(uint8_t group)
{
  uint8_t buf[6] = {0};
  uint8_t recvbuf[9] = {0};
  buf[0] = CMD_CHANGE_IIC_ADDR;
  buf[1] = group;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  delay(100);
  readData(0, recvbuf, 9);
  if (recvbuf[8] != FucCheckSum(recvbuf, 8))
    return 0;
  return recvbuf[2];
}

//IIC底层通信
DFRobot_GAS_I2C::DFRobot_GAS_I2C(TwoWire *pWire, uint8_t addr)
{
  _pWire = pWire;
  this->_I2C_addr = addr;
}

bool DFRobot_GAS_I2C::begin()
{
  _pWire->begin();
  _pWire->beginTransmission(_I2C_addr);
  if(_pWire->endTransmission() == 0)
    return true;
  else
    return false;
}

float DFRobot_GAS_I2C::setI2cAddr(uint8_t addr)
{
  this->_I2C_addr = addr;
}

bool DFRobot_GAS_I2C::dataIsAvailable()
{
  uint8_t buf[6] = {0};
  uint8_t recvbuf[9] = {0};
  buf[0] = CMD_GET_ALL_DTTA;
  sProtocol_t _protocol = pack(buf, sizeof(buf));
  writeData(0, (uint8_t *)&_protocol, sizeof(_protocol));
  readData(0, recvbuf, 9);
  if (recvbuf[8] != FucCheckSum(recvbuf, 8))
    return false;
  else{
    memcpy((uint8_t *)&AllData, recvbuf, 9);
    analysisAllData();
    return true;
  }
}

void DFRobot_GAS_I2C::writeData(uint8_t Reg ,void* pData ,uint8_t len)
{
  uint8_t* Data = (uint8_t *)pData;
  _pWire->beginTransmission(this->_I2C_addr);
  _pWire->write(Reg);
  for(uint8_t i = 0; i < len; i++)
  {
    _pWire->write(Data[i]);
  }
  _pWire->endTransmission();
}

int16_t DFRobot_GAS_I2C::readData(uint8_t Reg,uint8_t *Data,uint8_t len)
{
  int i=0;
  _pWire->beginTransmission(this->_I2C_addr);
  _pWire->write(Reg);
  if(_pWire->endTransmission() != 0)
  {
    return -1;
  }
  _pWire->requestFrom((uint8_t)this->_I2C_addr,(uint8_t)len);
  while (_pWire->available())
  {
    Data[i++]=_pWire->read();
  }
  return len;
}

//UART底层通信
#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
DFRobot_GAS_SoftWareUart::DFRobot_GAS_SoftWareUart(SoftwareSerial *psoftUart)
{
  _psoftUart = psoftUart;
}

bool DFRobot_GAS_SoftWareUart::begin()
{
  _psoftUart->begin(9600);
  return true;
}

bool DFRobot_GAS_SoftWareUart::dataIsAvailable()
{
  uint8_t len =_psoftUart->available();
  if (len>0)
  {
    readData(0,(uint8_t* )&AllData,len);
    analysisAllData();
    return true;
  }
  else
  {
    return false;
  }  
}

void DFRobot_GAS_SoftWareUart::writeData(uint8_t Reg, void *pData, uint8_t len)
{
  uint8_t* Data = (uint8_t* )pData;
  _psoftUart->write(Data,len);
}

int16_t DFRobot_GAS_SoftWareUart::readData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  uint32_t time = millis();
  uint8_t length;
  while((millis()-time)<3000)
  {
    length = _psoftUart->available();
    if (length == len)
      break;
  }
  int i=0;
  for (int i = Reg; i < length; i++)
  {
    Data[i] = _psoftUart->read();
    if (i>=8)
      return len;
  }
  return 0;
}

#else

DFRobot_GAS_HardWareUart::DFRobot_GAS_HardWareUart(HardwareSerial *phardUart)
{
  this->_pharduart = phardUart;
}

bool DFRobot_GAS_HardWareUart::begin()
{
  this->_pharduart->begin(9600);
  return true;
}

bool DFRobot_GAS_HardWareUart::dataIsAvailable()
{
  uint8_t len = _pharduart->available();
  if (len > 0)
  {
    readData(0, (uint8_t *)&AllData, len);
    analysisAllData();
    return true;
  }
  else
  {
    return false;
  }
}

void DFRobot_GAS_HardWareUart::writeData(uint8_t Reg, void *pData, uint8_t len)
{
  uint8_t *Data = (uint8_t *)pData;
  this->_pharduart->write(Data, len);
}

int16_t DFRobot_GAS_HardWareUart::readData(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  uint32_t time = millis();
  uint8_t length;
  while ((millis() - time) < 3000)
  {
    length = _pharduart->available();
    if (length == len)
      break;
  }

  int i = 0;
  for (int i = Reg; i < length; i++)
  {
    Data[i] = _pharduart->read();
    if (i >= 8)
      return len;
  }
  return 0;
}
#endif