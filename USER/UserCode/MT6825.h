#ifndef _MT6825_H
#define _MT6825_H
#include "sys.h"
#include "hardware.h"
//Encoder_Pan
#define Pan_SPI_CS_H   GPIO_SetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_NSS_Pin)
#define Pan_SPI_CS_L   GPIO_ResetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_NSS_Pin)
#define Pan_SPI_CK_H   GPIO_SetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_SCK_Pin)
#define Pan_SPI_CK_L   GPIO_ResetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_SCK_Pin)
#define Pan_SPI_DI_H   GPIO_SetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_MISO_Pin)
#define Pan_SPI_DI_L   GPIO_ResetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_MISO_Pin)
#define Pan_SPI_DO_H   GPIO_SetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_MOSI_Pin)
#define Pan_SPI_DO_L   GPIO_ResetBits(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_MOSI_Pin)
#define Pan_SPI_DI     GPIO_ReadInputDataBit(Pan_SPI_port.GPIOx,Pan_SPI_port.SPI_MISO_Pin)
#ifdef FOC
#define Tilt_SPI_CS_H    GPIO_SetBits(GPIOA,Tilt_SPI_port.SPI_NSS_Pin)
#define Tilt_SPI_CS_L    GPIO_ResetBits(GPIOA,Tilt_SPI_port.SPI_NSS_Pin)
#define Tilt_SPI_CK_H    GPIO_SetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_SCK_Pin)
#define Tilt_SPI_CK_L    GPIO_ResetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_SCK_Pin)
#define Tilt_SPI_DI_H    GPIO_SetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MISO_Pin)
#define Tilt_SPI_DI_L    GPIO_ResetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MISO_Pin)
#define Tilt_SPI_DO_H    GPIO_SetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MOSI_Pin)
#define Tilt_SPI_DO_L    GPIO_ResetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MOSI_Pin)
#define Tilt_SPI_DI      GPIO_ReadInputDataBit(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MISO_Pin)
#else
//Encoder_Tilt
#define Tilt_SPI_CS_H    GPIO_SetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_NSS_Pin)
#define Tilt_SPI_CS_L    GPIO_ResetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_NSS_Pin)
#define Tilt_SPI_CK_H    GPIO_SetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_SCK_Pin)
#define Tilt_SPI_CK_L    GPIO_ResetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_SCK_Pin)
#define Tilt_SPI_DI_H    GPIO_SetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MISO_Pin)
#define Tilt_SPI_DI_L    GPIO_ResetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MISO_Pin)
#define Tilt_SPI_DO_H    GPIO_SetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MOSI_Pin)
#define Tilt_SPI_DO_L    GPIO_ResetBits(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MOSI_Pin)
#define Tilt_SPI_DI      GPIO_ReadInputDataBit(Tilt_SPI_port.GPIOx,Tilt_SPI_port.SPI_MISO_Pin)
#endif
void MT6825_Encoder_SPI1_Init(void);
void MT6825_Encoder_SPI2_Init(void);
u32 MT6825_Angle_pan(void);
u32 MT6825_Pan(void);              //±àÂëÆ÷Ë®Æ½½Ç
u32 MT6825_Tilt(void);             //´¹Ö±±àÂëÆ÷½Ç¶È
#endif
