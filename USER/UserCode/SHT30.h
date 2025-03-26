#ifndef _SHT30_H
#define _SHT30_H
//#include "hardware.h"
#include "parameters.h"
//***********************************************************************************
//                                  temperature sensor
//***********************************************************************************
#define SHT30_GPIO      GPIOC
#define SHT30_RCC       RCC_AHB1Periph_GPIOC
#define SHT30_SDA_Pin   GPIO_Pin_12//PC12//PD4
#define SHT30_SCL_Pin   GPIO_Pin_11//PC11//PD3
#define SHT30_SDA_GPIO  SHT30_GPIO
#define SHT30_SCL_GPIO  SHT30_GPIO
#define SHT30_SDA_RCC   SHT30_RCC
#define SHT30_SCL_RCC   SHT30_RCC
#define SHT30_SDA_H     GPIO_SetBits(SHT30_SDA_GPIO,SHT30_SDA_Pin)
#define SHT30_SCL_H     GPIO_SetBits(SHT30_SCL_GPIO,SHT30_SCL_Pin)
#define SHT30_SDA_L     GPIO_ResetBits(SHT30_SDA_GPIO,SHT30_SDA_Pin)
#define SHT30_SCL_L     GPIO_ResetBits(SHT30_SCL_GPIO,SHT30_SCL_Pin)
//IO方向设置
#define SHT30_SDA_IN    SHT30_SDA_IO_SET(1)
#define SHT30_SDA_OUT   SHT30_SDA_IO_SET(0)
#define SHT30_Read_SDA  GPIO_ReadInputDataBit(SHT30_SDA_GPIO,SHT30_SDA_Pin)
#define SHT30_WriteCMD  0x00
#define SHT30_ReadCMD   0x01
#define SHT30_ADDR                  0x88    /* 从机地址 */
#define Period_Measurement_Command_Set  0x2130
#define Period_Measurement_Command_Read 0xe000
#define Status_Register_Command_Read    0xf32d
void SHT30_IIC_Init(void);
void SHT30_WriteData(u16 WriteAddr,u16 Command);
void SHT30_ReadData(u16 ReadAddr,u16 Command,u8 *data,u8 len);
#endif
