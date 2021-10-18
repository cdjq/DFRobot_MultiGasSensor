# -*- coding: utf-8 -*
""" 
  @file DFRobot_MultiGasSensor.py
  @note DFRobot_MultiGasSensor Class infrastructure, implementation of underlying methods
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @licence     The MIT License (MIT)
  @author      [PengKaixing](kaixing.peng@dfrobot.com)
  @version  V0.2
  @date  2021-03-31
  @get from https://www.dfrobot.com
  @url https://github.com/DFRobot/DFRobot_MultiGasSensor
"""
import serial
import time
import smbus
import spidev
import os
import math
import RPi.GPIO as GPIO

logger = logging.getLogger()
logger.setLevel(logging.INFO)  #显示所有的打印信息
#logger.setLevel(logging.FATAL)#如果不想显示过多打印，只打印错误，请使用这个选项
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter) 
logger.addHandler(ph)

I2C_MODE  = 0x01
UART_MODE = 0x02

sendbuf = [0]*9  
recvbuf = [0]*9
tempSwitch = 0
temp = 0.0
def fuc_check_sum(i,ln):
  '''!
    @brief CRC校验函数
    @param1 i  :CRC原始数据列表
    @param2 ln :长度
    @return CRC校验值
  '''
  tempq=0
  for j in range(1,ln-1):
    tempq+=i[j]
  tempq=(((~tempq)&0xff)+1)
  return tempq

def clear_buffer(buf,length):
  '''!
    @brief 列表数值置为0
    @param1 buf:等待清空的列表
    @param2 length :长度
  '''
  for i in range(0,length):
    buf[i]=0

class DFRobot_MultiGasSensor(object):
  '''!
    @brief 这是一个用于复杂环境中检测多种气体的传感器父类
    @details 用于检测 O2 CO H2S NO2 O3 CL2 NH3 H2 HCL
    @n       SO2 HF PH3等气体。只需要硬件切换对应的探头
    @n       就可以了。同时支持气体高阈值或者低阈值报警
    @n       功能
  '''  
  INITIATIVE =  0x03
  PASSIVITY  =  0x04
  O2         =  0x05
  CO         =  0x04
  H2S        =  0x03
  NO2        =  0x2C
  O3         =  0x2A
  CL2        =  0x31
  NH3        =  0x02
  H2         =  0x06
  HCL        =  0X2E
  SO2        =  0X2B
  HF         =  0x33
  PH3        =  0x45
  GASCON     =  0x00
  GASKIND    =  0x01
  ON         =  0x01
  OFF        =  0x00
  gasconcentration = 0.0
  gastype       =    ""  
  temp          =    0.0
  
  def __init__(self ,bus ,Baud):
    if bus != 0:
      self.i2cbus = smbus.SMBus(bus)
      self.__uart_i2c = I2C_MODE
    else:
      self.ser = serial.Serial("/dev/ttyAMA0" ,baudrate=Baud,stopbits=1, timeout=0.5)
      self.__uart_i2c = UART_MODE
      if self.ser.isOpen == False:
        self.ser.open()  
        
  def __getitem__(self, k):
    if k == recvbuf:
      return recvbuf

  def analysis_all_data(self,recv):
    '''!
      @brief   解析获取到的数据列表
      @param recv 获取到的数据
    '''    
    #recv[5]表示的是分辨率，0表示分辨率为：1，1表示分辨率为：0.1，2表示分辨率为：0.01
    if(recv[5]==0):
      self.gasconcentration = (recv[2] << 8) + recv[3]
    elif(recv[5]==1):
      self.gasconcentration = 0.1*((recv[2] << 8) + recv[3])
    elif(recv[5]==2):
      self.gasconcentration = 0.01*((recv[2] << 8) + recv[3]) 
    #recv[4]表示的是探头类型
    if recv[4]==0x05:
      self.gastype = "O2"
    elif recv[4]==0x04:
      self.gastype = "CO"
    elif recv[4]==0x03:
      self.gastype = "H2S"
    elif recv[4]==0x2C:
      self.gastype = "NO2"
    elif recv[4]==0x2A:
      self.gastype = "O3"
    elif recv[4]==0x31:
      self.gastype = "CL2"
    elif recv[4]==0x02:
      self.gastype = "NH3"
    elif recv[4]==0x06:
      self.gastype = "H2"
    elif recv[4]==0x33:
      self.gastype = "HF"
    elif recv[4]==0x45:
      self.gastype = "PH3"
    else:
      self.gastype =""
    Con = self.gasconcentration  
    if (self.gastype == self.O2):
      pass
    elif (self.gastype == self.CO) :
      if(((temp)>-20) and ((temp)<20)):
        Con = (Con/(0.005*(temp)+0.9))
      elif (((temp)>20) and ((temp)<40)):
        Con = (Con/(0.005*(temp)+0.9)-(0.3*(temp)-6))
      else:
        Con = 0.0
    elif (self.gastype == self.H2S):
      if(((temp)>-20) and ((temp)<20)):
        Con = (Con/(0.006*(temp)+0.92))
      elif (((temp)>20) and ((temp)<40)):
        Con = (Con/(0.006*(temp)+0.92)-(0.015*(temp)+2.4))
      else :
        Con = 0.0
    elif (self.gastype == self.NO2):
      if(((temp)>-20) and ((temp)<0)):
        Con = ((Con/(0.005*(temp)+0.9)-(-0.0025*(temp))))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/(0.005*(temp)+0.9)-(0.005*(temp)+0.005)))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/(0.005*(temp)+0.9)-(0.0025*(temp)+0.1)))
      else :
        Con =   0.0
    elif (self.gastype == self.O3):
      if(((temp)>-20) and ((temp)<0)):
        Con = ((Con/(0.015*(temp)+1.1)-0.05))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/1.1-(0.01*(temp))))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/1.1-(-0.05*(temp)+0.3)))
      else :
         Con = 0.0
    elif (self.gastype == self.CL2):
      if(((temp)>-20) and ((temp)<0)):
        Con = ((Con/(0.015*(temp)+1.1)-(-0.0025*(temp))))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/1.1-0.005*(temp)))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/1.1-(0.06*(temp)-0.12)))
      else:
        Con = 0.0
    elif (self.gastype ==self.NH3):
      if(((temp)>-20) and ((temp)<0)):
        Con = (Con/(0.08*(temp)+3.98)-(-0.005*(temp)+0.3))
      elif (((temp)>0) and ((temp)<20)):
        Con = (Con/(0.08*(temp)+3.98)-(-0.005*(temp)+0.3))
      elif (((temp)>20) and ((temp)<40)):
        Con = (Con/(0.004*(temp)+1.08)-(-0.1*(temp)+2))
      else:
        Con = 0.0
    elif (self.gastype == self.H2):
      if(((temp)>-20) and ((temp)<40)):
        Con = (Con/(0.74*(temp)+0.007)-5)
      else:
        Con =   0.0
    elif (self.gastype == self.HF):
      if(((temp)>-20) and ((temp)<0)):
        Con = (((Con/1)-(-0.0025*(temp))))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/1+0.1))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/1-(0.0375*(temp)-0.85)))
      else :
        Con = 0.0
    elif (self.gastype == self.PH3):
      if(((temp)>-20) and ((temp)<40)):
        Con = ((Con/(0.005*(temp)+0.9)))
    else:
      Con = 0.0             
    if(Con>0):
      self.gasconcentration = Con
    else:
      self.gasconcentration = 0
    temp_ADC=(recv[6]<<8)+recv[7] 
    Vpd3=float(temp_ADC/1024.0)*3
    Rth = Vpd3*10000/(3-Vpd3)
    self.temp = 1/(1/(273.15+25)+1/3380.13*(math.log(Rth/10000)))-273.15     
    
  def change_acquire_mode(self,mode):
    '''!
      @brief 改变传感器采集到气体以后数据上报到主控的方式
      @param mode 模式选择
      @n     INITIATIVE：传感器主动上报
      @n     PASSIVITY ：主控发送请求，传感器才能上报数据
      @return 返回改变气体模式是否成功
      @retval True   change success
      @retval False  change fail
    '''
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x78
    sendbuf[3]=mode
    sendbuf[4]=0x00
    sendbuf[5]=0x00
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    self.write_data(0,sendbuf,9)
    time.sleep(0.1)
    self.read_data(0,recvbuf,9)
    if(recvbuf[2]==1):
      return True
    else:
      return False

  def read_gas_concentration(self):
    '''!
      @brief 获取传感器获取的气体浓度或者气体的类型
      @return 如果数据传输正常，那么返回气体浓度；否则，返回0xffff
    '''  
    global sendbuf
    global recvbuf    
    clear_buffer(recvbuf,9)
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x86
    sendbuf[3]=0x00
    sendbuf[4]=0x00
    sendbuf[5]=0x00
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    self.write_data(0,sendbuf,9)
    time.sleep(0.1)
    self.read_data(0,recvbuf,9)
    if(fuc_check_sum(recvbuf,8) == recvbuf[8]):
      Con=((recvbuf[2]<<8)+recvbuf[3])*1.0
    else:
      return 0.0 
    #recv[4]表示的是探头类型
    if recvbuf[4]==0x05:
      self.gastype = "O2"
    elif recvbuf[4]==0x04:
      self.gastype = "CO"
    elif recvbuf[4]==0x03:
      self.gastype = "H2S"
    elif recvbuf[4]==0x2C:
      self.gastype = "NO2"
    elif recvbuf[4]==0x2A:
      self.gastype = "O3"
    elif recvbuf[4]==0x31:
      self.gastype = "CL2"
    elif recvbuf[4]==0x02:
      self.gastype = "NH3"
    elif recvbuf[4]==0x06:
      self.gastype = "H2"
    elif recvbuf[4]==0x33:
      self.gastype = "HF"
    elif recvbuf[4]==0x45:
      self.gastype = "PH3"
    else:
      self.gastype = ""
    gastype = recvbuf[4]
    decimal_digits = recvbuf[5]
    if(tempSwitch == self.OFF):
      if(decimal_digits==1):
        Con =  (Con*0.1)
      elif(decimal_digits==2):
        Con =  (Con*0.01)
      return Con      
    if (gastype == self.O2):
      pass
    elif (gastype == self.CO) :
      if(((temp)>-20) and ((temp)<20)):
        Con = (Con/(0.005*(temp)+0.9))
      elif (((temp)>20) and ((temp)<40)):
        Con = (Con/(0.005*(temp)+0.9)-(0.3*(temp)-6))
      else:
        Con = 0.0
    elif (gastype == self.H2S):
      if(((temp)>-20) and ((temp)<20)):
        Con = (Con/(0.006*(temp)+0.92))
      elif (((temp)>20) and ((temp)<40)):
        Con = (Con/(0.006*(temp)+0.92)-(0.015*(temp)+2.4))
      else :
        Con = 0.0
    elif (gastype == self.NO2):
      if(((temp)>-20) and ((temp)<0)):
        Con = ((Con/(0.005*(temp)+0.9)-(-0.0025*(temp))))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/(0.005*(temp)+0.9)-(0.005*(temp)+0.005)))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/(0.005*(temp)+0.9)-(0.0025*(temp)+0.1)))
      else :
        Con =   0.0
    elif (gastype == self.O3):
      if(((temp)>-20) and ((temp)<0)):
        Con = ((Con/(0.015*(temp)+1.1)-0.05))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/1.1-(0.01*(temp))))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/1.1-(-0.05*(temp)+0.3)))
      else :
         Con = 0.0
    elif (gastype == self.CL2):
      if(((temp)>-20) and ((temp)<0)):
        Con = ((Con/(0.015*(temp)+1.1)-(-0.0025*(temp))))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/1.1-0.005*(temp)))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/1.1-(0.06*(temp)-0.12)))
      else:
        Con = 0.0
    elif (gastype ==self.NH3):
      if(((temp)>-20) and ((temp)<0)):
        Con = (Con/(0.08*(temp)+3.98)-(-0.005*(temp)+0.3))
      elif (((temp)>0) and ((temp)<20)):
        Con = (Con/(0.08*(temp)+3.98)-(-0.005*(temp)+0.3))
      elif (((temp)>20) and ((temp)<40)):
        Con = (Con/(0.004*(temp)+1.08)-(-0.1*(temp)+2))
      else:
        Con = 0.0
    elif (gastype == self.H2):
      if(((temp)>-20) and ((temp)<40)):
        Con = (Con/(0.74*(temp)+0.007)-5)
      else:
        Con =   0.0
    elif (gastype == self.HF):
      if(((temp)>-20) and ((temp)<0)):
        Con = (((Con/1)-(-0.0025*(temp))))
      elif (((temp)>0) and ((temp)<20)):
        Con = ((Con/1+0.1))
      elif (((temp)>20) and ((temp)<40)):
        Con = ((Con/1-(0.0375*(temp)-0.85)))
      else :
        Con = 0.0
    elif (gastype == self.PH3):
      if(((temp)>-20) and ((temp)<40)):
        Con = ((Con/(0.005*(temp)+0.9)))
    else:
      Con = 0.0 
    if(Con<0):
      return 0
    else:
      return Con     

  def read_gas_type(self):
    '''!
      @brief 获取传感器获取气体的类型
      @return 气体类型
      @n  O2   0x05
      @n  CO   0x04
      @n  H2S  0x03
      @n  NO2  0x2C
      @n  O3   0x2A
      @n  CL2  0x31
      @n  NH3  0x02
      @n  H2   0x06
      @n  HCL  0X2E
      @n  SO2  0X2B
      @n  HF   0x33
      @n  PH3  0x45
    '''  
    clear_buffer(recvbuf,9)
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x86
    sendbuf[3]=0x00
    sendbuf[4]=0x00
    sendbuf[5]=0x00
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    write_data(0,sendbuf,9)
    time.sleep(0.1)
    read_reg(0,recvbuf,9)
    if(fuc_check_sum(recvbuf,8) == recvbuf[8]):
      return (recvbuf[4])
    else:
      return 0xff   
    
  def set_threshold_alarm(self,switchof,threshold,gasType):
    '''!
      @brief 设置传感器报警的阈值
      @param1 switchof 设置是否打开报警
      @n        ON    打开报警功能
      @n        OFF   关闭报警功能
      @param2 threshold 设置报警的阈值
      @param3 gasType 气体类型
      @return 设置阈值报警是否成功
      @retval True   change success
      @retval False  change fail
    '''  
    if (gasType == self.O2):
      threshold *= 10
    elif (gasType == self.NO2):
      threshold *= 10
    elif (gasType == self.O3):
      threshold *= 10
    elif (gasType == self.CL2):
      threshold *= 10
    elif (gasType == self.HCL):
      threshold *= 10
    elif (gasType == self.SO2):
      threshold *= 10
    elif (gasType == self.HF):
      threshold *= 10
    elif (gasType == self.PH3):
      threshold *= 10
    global sendbuf
    global recvbuf  
    clear_buffer(recvbuf,9)
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x89
    sendbuf[3]=switchof
    sendbuf[4]=threshold>>8
    sendbuf[5]=threshold
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    self.write_data(0,sendbuf,9)
    time.sleep(0.1)
    self.read_data(0,recvbuf,9)
    if (recvbuf[8]!=fuc_check_sum(recvbuf,8)):
      return False
    if(recvbuf[2]==1):
      return True
    else:
      return False   

  def read_temp(self):
    '''!
      @brief 获取传感器的板载温度
      @return 板子温度 单位为摄氏度
    '''
    clear_buffer(recvbuf,9)
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x87
    sendbuf[3]=0x00
    sendbuf[4]=0x00
    sendbuf[5]=0x00
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    self.write_data(0,sendbuf,9)
    time.sleep(0.1)
    self.read_data(0,recvbuf,9)
    temp_ADC=(recvbuf[2]<<8)+recvbuf[3]
    Vpd3=float(temp_ADC/1024.0)*3
    Rth = Vpd3*10000/(3-Vpd3)
    Tbeta = 1/(1/(273.15+25)+1/3380.13*(math.log(Rth/10000)))-273.15
    return Tbeta
    
  def set_temp_compensation(self,tempswitch):
    '''!
      @brief 设置是否开启温度补偿，传感器在不同温度下的输出值会有差别，所以
      @n     为了获取到的气体浓度更精确，在计算气体浓度的时候需要增加温度补偿
      @param tempswitch：温度补偿开关
                   ON  ：打开温度补偿
                   OFF ：关闭温度补偿
    '''  
    tempSwitch = tempswitch
    temp = self.read_temp()
    
  def read_volatage_data(self):
    '''!
      @brief 获取传感器气体浓度以原始电压输出，不同于直接读取传感器寄存器，这
      @n     个函数主要用来检验读取的气体浓度是否准确
      @param  vopin：用来接收传感器探头原始电压输出的引脚
      @return 传感器气体浓度的原始电压输出
    '''
    clear_buffer(recvbuf,9)
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x91
    sendbuf[3]=0x00
    sendbuf[4]=0x00
    sendbuf[5]=0x00
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    self.write_data(0,sendbuf,9)
    time.sleep(0.1)
    self.read_data(0,recvbuf,9)
    if (recvbuf[8] != fuc_check_sum(recvbuf, 8)):
      return 0.0
    else:
      return (((recvbuf[2] << 8) + recvbuf[3])*3.0/1024*2);

  def change_i2c_addr_group(self,group):
    '''!
      @brief 改变IIC地址组
      @param  group：想要使传感器变成的组号
    '''   
    global sendbuf
    global recvbuf
    clear_buffer(recvbuf,9)
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x92
    sendbuf[3]=group
    sendbuf[4]=0x00
    sendbuf[5]=0x00
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    self.write_data(0,sendbuf,9)      
    time.sleep(0.1)
    self.read_data(0,recvbuf,9)  
    if (recvbuf[8] != fuc_check_sum(recvbuf, 8)):
      return False
    else:
      return recvbuf[2]    
      

class DFRobot_MultiGasSensor_I2C(DFRobot_MultiGasSensor):

  def __init__(self ,bus ,addr):
    self.__addr = addr
    super(DFRobot_MultiGasSensor_I2C, self).__init__(bus,0)

  def data_is_available(self):
    '''
      * @brief IIC在主动模式下调用此函数，用以判断数据线上有没有数据
      * @return 传感器是否有数据上传
      * @retval True  success is Available
      * @retval False  error is unavailable
      *
    ''' 
    clear_buffer(recvbuf,9)
    sendbuf[0]=0xff
    sendbuf[1]=0x01
    sendbuf[2]=0x88
    sendbuf[3]=0x00
    sendbuf[4]=0x00
    sendbuf[5]=0x00
    sendbuf[6]=0x00
    sendbuf[7]=0x00
    sendbuf[8]=fuc_check_sum(sendbuf,8)
    self.write_data(0,sendbuf,9)
    time.sleep(0.1)
    self.read_data(0,recvbuf,9)  
    if (recvbuf[8] == fuc_check_sum(recvbuf, 8)):
      self.analysis_all_data(recvbuf)
      return True
    else:
      return False

  def write_data(self, reg, data , length):
    '''
      @brief writes data to a register
      @param1 reg register address
      @param2 value written data
    '''  
    while 1:
      try:
        self.i2cbus.write_i2c_block_data(self.__addr ,reg ,data)
        return
      except:
        print("please check connect!")
        time.sleep(1)
        return

  def read_data(self, reg ,data,length):
    '''
      @brief read the data from the register
      @param1 reg register address
      @param2 value read data
    '''
    try:
      rslt = self.i2cbus.read_i2c_block_data(self.__addr ,reg , length)
    except:
      rslt = -1
    recvbuf=rslt
    return length
    
class DFRobot_MultiGasSensor_UART(DFRobot_MultiGasSensor):
  '''
    @brief An example of an UART interface module
  '''
  def __init__(self ,Baud):
    self.__Baud = Baud
    try:
      super(DFRobot_MultiGasSensor_UART, self).__init__(0,Baud)
    except:
      print ("plese get root!")
    
  def data_is_available(self):  
    '''
      *@brief uart在主动模式下调用此函数，用以判断数据线上有没有数据
      *@return 传感器是否有数据上传
      *@retval  True  success is Available
      *@retval  False  error is unavailable
    '''    
    if(self.read_data(0,recvbuf,9)==9):
      if(fuc_check_sum(recvbuf,8) == recvbuf[8]):
        self.analysis_all_data(recvbuf)
        return True
      else:
        return False
    else:
      return False
        
  def write_data(self, reg, data , length): 
    self.ser.write(data)
    time.sleep(1)
    
  def read_data(self, reg ,data,length):
    global recvbuf
    timenow = time.time()
    i = 0
    while(time.time() - timenow) <= 2:
      count = self.ser.inWaiting()
      if count != 0:
        recvbuf = self.ser.read(count)
        self.ser.flushInput()
        recvbuf =[ord(c) for c in recvbuf]
        return count
  