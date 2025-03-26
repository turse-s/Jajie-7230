#include "MT6825.h"
#include "delay.h"
#include "parameters.h"
//#define Pan_SSI_CS_H   GPIO_SetBits(SPI1_NSS_GPIO,SPI1_NSS_Pin)
//#define Pan_SSI_CS_L   GPIO_ResetBits(SPI1_NSS_GPIO,SPI1_NSS_Pin)
//#define Pan_SSI_CK_H   GPIO_SetBits(SPI1_GPIO,SPI1_SCK_Pin)
//#define Pan_SSI_CK_L   GPIO_ResetBits(SPI1_GPIO,SPI1_SCK_Pin)
//#define Pan_SSI_DI_H   GPIO_SetBits(SPI1_GPIO,SPI1_MISO_Pin)
//#define Pan_SSI_DI_L   GPIO_ResetBits(SPI1_GPIO,SPI1_MISO_Pin)
//#define Pan_SSI_DO_H   GPIO_SetBits(SPI1_GPIO,SPI1_MOSI_Pin)
//#define Pan_SSI_DO_L   GPIO_ResetBits(SPI1_GPIO,SPI1_MOSI_Pin)
//#define Pan_SSI_DI     GPIO_ReadInputDataBit(SPI1_GPIO,SPI1_MISO_Pin)
//void MT6825_SPI1_Init(u16 SPI_psc)
//{
//    GPIO_InitTypeDef  GPIO_InitStructure;
//    SPI_InitTypeDef  SPI_InitStructure;
//
//    RCC_APB2PeriphClockCmd(SPI1_CS_RCC|RCC_APB2Periph_SPI1, ENABLE);
//    //NSS
//    GPIO_InitStructure.GPIO_Pin = SPI1_CS_Pin;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
//    GPIO_Init(SPI1_CS_GPIO,&GPIO_InitStructure);
//
//    //SPI1
//    GPIO_InitStructure.GPIO_Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MOSI_Pin;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用功能
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
//    GPIO_Init(SPI1_CS_GPIO,&GPIO_InitStructure);
//
//
//    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
//    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;     //设置SPI工作模式:设置为主SPI
//    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;     //设置SPI的数据大小:SPI发送接收8位帧结构
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;       //串行同步时钟的空闲状态为高电平
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;  //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
//    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;     //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
//    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_psc;        //定义波特率预分频的值:波特率预分频值为256
//    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;    //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
//    SPI_InitStructure.SPI_CRCPolynomial = 7;  //CRC值计算的多项式
//    SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
//    SPI_Cmd(SPI1, ENABLE); //使能SPI外设
//      SPI1_CS_H;
////        SPI1_ReadWriteByte(0xff);//启动传输
//}
//u8 MT6825_SPI1_ReadWriteByte(u8 TxData)
//{
//    u16 data;
//
//
//    //SPI1_NSS_L;
//    //Delay_xUS(5);
////    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空
//    SPI_I2S_SendData(SPI1,TxData); //通过外设SPIx发送一个byte  数据
//    while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空
////    delay_us(2);
////    SPI1_NSS_H;
////    Delay_xUS(5);
////        SPI1_NSS_L;
////        Delay_xUS(5);
//    while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完一个byte
//    data = SPI_I2S_ReceiveData(SPI1);
//    //Delay_xUS(5);
//    //SPI1_NSS_H;
//    //Delay_xUS(5);
//    return data; //返回通过SPIx最近接收的数据
//}
/***********************************
*功能：读MT6825编码器器
***********************************/
//u32 MT6825_Angle_pan(void)
//{
//  u8 code=0;
//  u32 encode=0;
//  SPI1_CS_L;
//  Delay_xUS(5);
//  code=(u8)MT6825_SPI1_ReadWriteByte(0x83);
//  encode=(u8)MT6825_SPI1_ReadWriteByte(0x00);
//  code=(u8)MT6825_SPI1_ReadWriteByte(0x00);
//  encode=(encode<<8)+(code>>2);
//  code=(u8)MT6825_SPI1_ReadWriteByte(0x00);
//  encode=(encode<<8)+(code>>4);
//  Delay_xUS(5);
//  SPI1_CS_H;
//  Delay_xUS(5);
//  return encode;
//}
//EncoderPortInitialize
//void MT6825_Encoder_SPI1_Init(void)
//{
//    GPIO_InitTypeDef  GPIO_InitStructure;
//
//    RCC_APB2PeriphClockCmd(SPI1_CS_RCC |RCC_APB2Periph_SPI1, ENABLE);
//    //NSS
//    GPIO_InitStructure.GPIO_Pin = SPI1_CS_Pin;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
//    GPIO_Init(SPI1_CS_GPIO,&GPIO_InitStructure);
//
//    //SPI1
//    GPIO_InitStructure.GPIO_Pin = SPI1_SCK_Pin | SPI1_MOSI_Pin;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//复用功能
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
//    GPIO_Init(SPI1_SCK_GPIO,&GPIO_InitStructure);
//
//  GPIO_InitStructure.GPIO_Pin = SPI1_MISO_Pin;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(SPI1_SCK_GPIO,&GPIO_InitStructure);
//}
//void MT6825_Encoder_SPI2_Init(void)
//{
//   GPIO_InitTypeDef  GPIO_InitStructure;
//
//   RCC_APB2PeriphClockCmd(SPI2_CS_RCC, ENABLE);
//    //NSS
//   GPIO_InitStructure.GPIO_Pin = SPI2_CS_Pin;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
//   GPIO_Init(SPI2_CS_GPIO,&GPIO_InitStructure);
//
//    //SPI1
//   GPIO_InitStructure.GPIO_Pin = SPI2_SCK_Pin | SPI2_MOSI_Pin;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//复用功能
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
//   GPIO_Init(SPI2_SCK_GPIO,&GPIO_InitStructure);
//
//  GPIO_InitStructure.GPIO_Pin = SPI2_MISO_Pin;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//  GPIO_Init(SPI2_SCK_GPIO,&GPIO_InitStructure);
//}
int p_flag = 0,t_flag = 0;
u32 Encoder_MT6825_Pan(u32 byte)
{
    //读mt6825时，spi模式3（CPOL=1, CPHA=1），即空闲时clk=1，在clk上升沿读数
    u8 i;
    u32 p_data = 0;
    Pan_SPI_CK_H;
    Pan_SPI_CS_L;
    delay_us(1);
    for(i = 0; i < 32; i ++)
    {
        Pan_SPI_CK_L;
        if(byte & 0x80000000)
        {
            Pan_SPI_DO_H;
        }
        else
        {
            Pan_SPI_DO_L;
        }
        delay_us(1);
        byte <<= 1;
        Pan_SPI_CK_H;
        delay_us(1);
        p_data <<= 1;
        if(Pan_SPI_DI)
        {
            p_data ++;
        }
    }
    Pan_SPI_CS_H;
    if(p_data != 0xffffffff)
    {
        p_data = p_data & 0x00ffffff;
        if(((p_data >> 9) & 0x1) == 0)
        {
            p_data = p_data >> 4;
            p_data = ((p_data >> 2) & 0xfffffff0)+(p_data & 0x0000000f);
            Pan_Encoder_State = Encoder_Normal;
        }
        else
        {
            p_data = p_data >> 4;
            p_data = ((p_data >> 2) & 0xfffffff0)+(p_data & 0x0000000f);
            p_data = p_data | 0xee000000;
            Pan_Encoder_State = Encoder_Error;
        }
    }
    else
    {
        Pan_Encoder_State = Encoder_Drop;
    }
    return p_data;
}
u32 Encoder_MT6825_Tilt(u32 byte)
{
    //读mt6825时，spi模式3（CPOL=1, CPHA=1），即空闲时clk=1，在clk上升沿读数
    u8 i;
    u32 p_data = 0;
    Tilt_SPI_CK_H;
    Tilt_SPI_CS_L;
    delay_us(1);
    for(i = 0; i < 32; i ++)
    {
        Tilt_SPI_CK_L;
        if(byte & 0x80000000)
        {
            Tilt_SPI_DO_H;
        }
        else
        {
            Tilt_SPI_DO_L;
        }
        delay_us(1);
        byte <<= 1;
        Tilt_SPI_CK_H;
        delay_us(2);
        p_data <<= 1;
        if(Tilt_SPI_DI)
        {
            p_data ++;
        }
    }
    Tilt_SPI_CS_H;
    if(p_data != 0xffffffff)
    {
        p_data = p_data & 0x00ffffff;
        if(((p_data >> 9) & 0x1) == 0)
        {
            p_data = p_data >> 4;
            p_data = ((p_data >> 2) & 0xfffffff0)+(p_data & 0x0000000f);
            Tilt_Encoder_State = Encoder_Normal;
        }
        else
        {
            p_data = p_data >> 4;
            p_data = ((p_data >> 2) & 0xfffffff0)+(p_data & 0x0000000f);
            p_data = p_data | 0xee000000;
            Tilt_Encoder_State = Encoder_Error;
        }
    }
    else
    {
         Tilt_Encoder_State = Encoder_Drop;
    }
    //Tilt_SPI_CS_H;
    return p_data;
}
u32 MT6825_Pan(void)
{
    return Encoder_MT6825_Pan(0x83000000);
}
u32 MT6825_Tilt(void)
{
    return Encoder_MT6825_Tilt(0x83000000);
}
