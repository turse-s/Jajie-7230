#ifndef _AS5600_H
#define _AS5600_H
#include "parameters.h"
/*水平编码器的引脚配置*/
#define Pan_I2C_SCL_GPIO    GPIOB
#define Pan_I2C_SCL_RCC     RCC_AHB1Periph_GPIOB
#define Pan_I2C_SCL_Pin     GPIO_Pin_6
#define Pan_I2C_SDA_GPIO    GPIOB
#define Pan_I2C_SDA_RCC     RCC_AHB1Periph_GPIOB
#define Pan_I2C_SDA_Pin     GPIO_Pin_7
/*
垂直编码器的引脚配置
*/
#define Tilt_I2C_SCL_GPIO    GPIOB
#define Tilt_I2C_SCL_RCC     RCC_AHB1Periph_GPIOB
#define Tilt_I2C_SCL_Pin     GPIO_Pin_8
#define Tilt_I2C_SDA_GPIO    GPIOB
#define Tilt_I2C_SDA_RCC     RCC_AHB1Periph_GPIOB
#define Tilt_I2C_SDA_Pin     GPIO_Pin_9
////IO方向设置
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}   //PB9输入模式
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
////IO操作函数
////#define IIC_SCL    PBout(8) //SCL
////#define IIC_SDA    PBout(9) //SDA
////#define READ_SDA   PBin(9)  //输入SDA
//IO操作函数
#define Pan_IIC_SCL    PBout(6)         //SCL
#define Pan_IIC_SDA    PBout(7)         //SDA    
#define Pan_READ_SDA   PBin(7)          //输入SDA 
#define Tilt_IIC_SCL   PBout(8)     //SCL
#define Tilt_IIC_SDA   PBout(9)    //SDA输出
#define Tilt_READ_SDA  PBin(9)    //SDA输入
//IIC所有操作函数
//IO方向设置
void Pan_SDA_IN(void);
void Pan_SDA_OUT(void);
void Tilt_SDA_IN(void);
void Tilt_SDA_OUT(void);
void Pan_IIC_Init(void);                //初始化IIC的IO口
void Pan_IIC_Start(void);               //发送IIC开始信号
void Pan_IIC_Stop(void);                //发送IIC停止信号
void Pan_IIC_Send_Byte(u8 txd);         //IIC发送一个字节
u8 Pan_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 Pan_IIC_Wait_Ack(void);              //IIC等待ACK信号
void Pan_IIC_Ack(void);                 //IIC发送ACK信号
void Pan_IIC_NAck(void);                //IIC不发送ACK信号
void Tilt_IIC_Init(void);       //初始化IIC的IO口
void Tilt_IIC_Start(void);      //发送IIC开始信号
void Tilt_IIC_stop(void);       //发送IIC停止信号
void Tilt_IIC_Send_Byte(u8 txd); //IIC发送一个字节
u8 Tilt_IIC_Read_Byte(unsigned char ack);  //IIC读取一个字节
u8 Tilt_IIC_Wait_ACK(void);          //IIC等待应答信号
void Tilt_IIC_Ack(void);            //IIC发送应答信号
void Tilt_IIC_NAck(void);           //IIC不发送应该信号
u16 Pan_get_Angle(void);           //读取水平角度
u16 Tilt_get_Angle(void);          //读取垂直角度
#endif
