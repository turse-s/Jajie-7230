#ifndef _24LCxx_H
#define _24LCxx_H
#include "parameters.h"
//********************************************
//EEPROM器件类型(bytes)
//********************************************
#define AT24C01     128
#define AT24C02     256
#define AT24C04     512
#define AT24C08     1024
#define AT24C16     2048
#define AT24C32     4096
#define AT24C64     8192
#define AT24C128    16384
#define AT24C256    32768
#define EE_TYPE AT24C128
//********************************************
//EEPROM器件地址
//********************************************
#define EEPRom_Device(n)    (u8)(n << 1)
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define EEPRom_Addr         EEPRom_Device(0)//更改器件地址,从0开始 ++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define EEPRom_WriteCMD     0xA0
#define EEPRom_ReadCMD      0xA1
//********************************************
//EEPROM硬件接口
//********************************************
#ifdef FOC
#define EE_SDA_Pin  GPIO_Pin_7//PB7
#define EE_SCL_Pin  GPIO_Pin_6//PB6
//#define EE_WP_Pin   GPIO_Pin_15//PE15
#define EE_GPIO     GPIOB
#define EE_SDA_GPIO EE_GPIO
#define EE_SCL_GPIO EE_GPIO
//#define EE_WP_GPIO  GPIOE
#define EE_RCC      RCC_AHB1Periph_GPIOB
#define EE_SDA_RCC  EE_RCC
#define EE_SCL_RCC  EE_RCC
//#define EE_WP_RCC   RCC_AHB1Periph_GPIOE
#define EE_SDA_H    GPIO_SetBits(EE_SDA_GPIO,EE_SDA_Pin)
#define EE_SCL_H    GPIO_SetBits(EE_SCL_GPIO,EE_SCL_Pin)
//#define EE_WP_H     GPIO_SetBits(EE_WP_GPIO,EE_WP_Pin)
#define EE_SDA_L    GPIO_ResetBits(EE_SDA_GPIO,EE_SDA_Pin)
#define EE_SCL_L    GPIO_ResetBits(EE_SCL_GPIO,EE_SCL_Pin)
//#define EE_WP_L     GPIO_ResetBits(EE_WP_GPIO,EE_WP_Pin)
//IO方向设置
#define EE_SDA_IN  EEPRom_SDA_IO_SET(1)
#define EE_SDA_OUT EEPRom_SDA_IO_SET(0)
#define EE_Read_SDA   GPIO_ReadInputDataBit(EE_SDA_GPIO,EE_SDA_Pin)
//EEPROM验证
#define EE_CRC_TIMES    10
#else
#define EE_SDA_Pin  GPIO_Pin_11//PB11
#define EE_SCL_Pin  GPIO_Pin_10//PB10
#define EE_WP_Pin   GPIO_Pin_15//PE15
#define EE_GPIO     GPIOB
#define EE_SDA_GPIO EE_GPIO
#define EE_SCL_GPIO EE_GPIO
#define EE_WP_GPIO  GPIOE
#define EE_RCC      RCC_AHB1Periph_GPIOB
#define EE_SDA_RCC  EE_RCC
#define EE_SCL_RCC  EE_RCC
#define EE_WP_RCC   RCC_AHB1Periph_GPIOE
#define EE_SDA_H    GPIO_SetBits(EE_SDA_GPIO,EE_SDA_Pin)
#define EE_SCL_H    GPIO_SetBits(EE_SCL_GPIO,EE_SCL_Pin)
#define EE_WP_H     GPIO_SetBits(EE_WP_GPIO,EE_WP_Pin)
#define EE_SDA_L    GPIO_ResetBits(EE_SDA_GPIO,EE_SDA_Pin)
#define EE_SCL_L    GPIO_ResetBits(EE_SCL_GPIO,EE_SCL_Pin)
#define EE_WP_L     GPIO_ResetBits(EE_WP_GPIO,EE_WP_Pin)
//IO方向设置
#define EE_SDA_IN  EEPRom_SDA_IO_SET(1)
#define EE_SDA_OUT EEPRom_SDA_IO_SET(0)
#define EE_Read_SDA   GPIO_ReadInputDataBit(EE_SDA_GPIO,EE_SDA_Pin)
//EEPROM验证
#define EE_CRC_TIMES    10
#endif
//********************************************
//EEPROM数据传递函数
//********************************************
void EEPRom_IIC_Init(void);
void EEPRom_SDA_IO_SET(u8 iotype);
void EEPRom_IIC_Start(void);
void EEPRom_IIC_Stop(void);
u8 EEPRom_Wait_Ack(void);
void EEPRom_Ack(void);
void EEPRom_nAck(void);
void EEPRom_TXD_Byte(u8 data);
u8 EEPRom_RXD_Byte(u8 ack);
void EEPRom_WriteDataLen(u8 DeviceAddr,u16 WriteAddr,u8 *DataToWrite,u16 len);
long EEPRom_ReadDataLen(u8 DeviceAddr,u16 WriteAddr,u8 *DataToRead,u16 len);
void EEPRom_WriteOneByte(u8 DeviceAddr,u16 WriteAddr,u8 DataToWrite);
u8 EEPRom_ReadOneByte(u8 DeviceAddr,u16 ReadAddr);
void EEPRom_WriteData(u16 WriteAddr,u32 DataToWrite);
u32 EEPRom_ReadData(u16 ReadAddr,u8 Len);
#endif
