#include "24LCxx.h"
#include "CRC16.h"
#include "parameters.h"
int Err_times = EE_CRC_TIMES;
//*****************************
//初始化EEPRom
#ifdef FOC
void EEPRom_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(EE_RCC,ENABLE);
    //配置管脚状态
    GPIO_InitStructure.GPIO_Pin = EE_SDA_Pin | EE_SCL_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(EE_GPIO, &GPIO_InitStructure);
    EE_SDA_H;
    EE_SCL_H;
}
#else
void EEPRom_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(EE_RCC,ENABLE);
    //配置管脚状态
    GPIO_InitStructure.GPIO_Pin = EE_SDA_Pin | EE_SCL_Pin | EE_WP_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(EE_GPIO, &GPIO_InitStructure);
    EE_SDA_H;
    EE_SCL_H;
    EE_WP_L;
}
#endif
//*****************************
//设置EEPRom_SDA方向,iotype 0:输出，1：输入
void EEPRom_SDA_IO_SET(u8 iotype)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(EE_SDA_RCC,ENABLE);
    switch(iotype)
    {
    case 0:
    {
        GPIO_InitStructure.GPIO_Pin = EE_SDA_Pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    }
    break;
    case 1:
    {
        GPIO_InitStructure.GPIO_Pin = EE_SDA_Pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    }
    break;
    }
    GPIO_Init(EE_SDA_GPIO,&GPIO_InitStructure);
}
//*****************************
//IIC起始信号
void EEPRom_IIC_Start(void)
{
    EE_SDA_OUT;
    EE_SDA_H;
    EE_SCL_H;
    delay_us(4);
    EE_SDA_L;
    delay_us(4);
    EE_SCL_L;
}
//*****************************
//IIC停止信号
void EEPRom_IIC_Stop(void)
{
    EE_SDA_OUT;
    EE_SCL_L;
    EE_SDA_L;
    EE_SCL_H;
    delay_us(4);
    EE_SDA_H;
    delay_us(4);
}
//*******************************************
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 EEPRom_Wait_Ack(void)
{
    u8 errtime;
    EE_SDA_IN;
    EE_SDA_H;
    delay_us(2);
    EE_SCL_H;
    delay_us(2);
    while(EE_Read_SDA)
    {
        errtime ++;
        if(errtime > 200)
        {
            EEPRom_IIC_Stop();
            return 1;
        }
    }
    EE_SCL_L;
    delay_us(2);
    return 0;
}
//*******************************************
//产生应答
void EEPRom_Ack(void)
{
    EE_SCL_L;
    EE_SDA_OUT;
    EE_SDA_L;
    delay_us(2);
    EE_SCL_H;
    delay_us(2);
    EE_SCL_L;
}
//*******************************************
//产生应答
void EEPRom_nAck(void)
{
    EE_SCL_L;
    EE_SDA_OUT;
    EE_SDA_H;
    delay_us(2);
    EE_SCL_H;
    delay_us(2);
    EE_SCL_L;
}
//*****************************
//发送字节
void EEPRom_TXD_Byte(u8 data)
{
    u8 i;
    EE_SDA_OUT;
    EE_SCL_L;
    delay_us(5);
    for(i = 0; i < 8; i ++)
    {
        if(data & 0x80)
        {
            EE_SDA_H;
        }
        else
        {
            EE_SDA_L;
        }
        data <<= 1;
        delay_us(2);
        EE_SCL_H;
        delay_us(2);
        EE_SCL_L;
        delay_us(5);
    }
}
//*****************************
//读取字节
u8 EEPRom_RXD_Byte(u8 ack)
{
    u8 i,receive;
    EE_SDA_IN;
    delay_us(5);
    for(i = 0; i < 8; i ++)
    {
        EE_SCL_L;
        delay_us(2);
        EE_SCL_H;
        receive <<= 1;
        if(EE_Read_SDA)
        {
            receive ++;
        }
        delay_us(2);
    }
    if(ack)
    {
        EEPRom_Ack();
    }
    else
    {
        EEPRom_nAck();
    }
    return receive;
}
//*******************************************
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址
//DataToWrite:要写入的数据
void EEPRom_WriteOneByte(u8 DeviceAddr,u16 WriteAddr,u8 DataToWrite)
{
    CPU_SR_ALLOC();
    OS_CRITICAL_ENTER();//进入临界区
    EEPRom_IIC_Start();
    if(EE_TYPE > AT24C16)
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1));       //发送写命令
        EEPRom_Wait_Ack();
        EEPRom_TXD_Byte(WriteAddr >> 8);//发送高地址
    }
    else
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1) + ((WriteAddr / 256) << 1));   //发送器件地址0xA0,写数据
    }
    EEPRom_Wait_Ack();
    EEPRom_TXD_Byte(WriteAddr & 0xff);   //发送低地址
    EEPRom_Wait_Ack();
    EEPRom_TXD_Byte(DataToWrite);     //发送字节
    EEPRom_Wait_Ack();
    EEPRom_IIC_Stop();//产生一个停止条件
    delay_us(10000);
    OS_CRITICAL_EXIT(); //退出临界区
}
//*******************************************
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址
//返回值  :读到的数据
u8 EEPRom_ReadOneByte(u8 DeviceAddr,u16 ReadAddr)
{
    u8 temp=0;
    EEPRom_IIC_Start();
    if(EE_TYPE > AT24C16)
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1));      //发送写命令
        EEPRom_Wait_Ack();
        EEPRom_TXD_Byte(ReadAddr >> 8);//发送高地址
    }
    else
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1) + ((ReadAddr / 256) << 1));   //发送器件地址0XA0,写数据
    }
    EEPRom_Wait_Ack();
    EEPRom_TXD_Byte(ReadAddr & 0xff);   //发送低地址
    EEPRom_Wait_Ack();
    EEPRom_IIC_Start();
    EEPRom_TXD_Byte(EEPRom_ReadCMD + (DeviceAddr << 1));           //进入接收模式
    EEPRom_Wait_Ack();
    temp = EEPRom_RXD_Byte(0);//不应答
    EEPRom_IIC_Stop();//产生一个停止条件
    return temp;
}
//*******************************************
//从某一地址连续写入len个字节
u8 ack_w[10];
void EEPRom_WriteDataLen(u8 DeviceAddr,u16 WriteAddr,u8 *DataToWrite,u16 len)
{
    u16 i = 0;
    u8 DataToRead[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    u8 DataToCompare[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//    if(WriteAddr > 0)//预置位必须从1号开始
//    {
    EEPRom_IIC_Start();
    if(EE_TYPE > AT24C16)
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1));       //发送写命令
        EEPRom_Wait_Ack();
        EEPRom_TXD_Byte(((WriteAddr - 1) * len) >> 8);//发送高地址
    }
    else
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1) + ((((WriteAddr - 1) * len) / 256) << 1));   //发送器件地址0xA0,写数据
    }
    EEPRom_Wait_Ack();
    EEPRom_TXD_Byte(((WriteAddr - 1) * len) & 0xff);   //发送低地址
    EEPRom_Wait_Ack();
    for(i = 0; i < len; i ++)
    {
        EEPRom_TXD_Byte(DataToWrite[i]);     //发送字节
        DataToCompare[i] = *(DataToWrite + i);
        ack_w[i] = EEPRom_Wait_Ack();
    }
    EEPRom_IIC_Stop();//产生一个停止条件
    delay_us(10000);
    if (!DeviceAddr)
    {
        EEPRom_ReadDataLen(DeviceAddr,WriteAddr,DataToRead,len);
        for (i = 0; i < EE_CRC_TIMES; i ++)
        {
            if (CRC_16(DataToRead,len) == CRC_16(DataToCompare,len))
            {
                break;
            }
        }
        if (i >= EE_CRC_TIMES)
        {
            Error_Record = 1;
        }
    }
//    }
}
//*******************************************
//从某一地址连续读出len个字节
long EEPRom_ReadDataLen(u8 DeviceAddr,u16 ReadAddr,u8 *DataToRead,u16 len)
{
    u8 i;
    u8 DataToCompare[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
//    if(ReadAddr > 0)//预置位必须从1号开始
//    {
    EEPRom_IIC_Start();
    if(EE_TYPE > AT24C16)
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1));      //发送写命令
        EEPRom_Wait_Ack();
        EEPRom_TXD_Byte(((ReadAddr - 1) * len) >> 8);//发送高地址
    }
    else
    {
        EEPRom_TXD_Byte(EEPRom_WriteCMD + (DeviceAddr << 1) + ((((ReadAddr - 1) * len) / 256) << 1));   //发送器件地址0XA0,写数据
    }
    EEPRom_Wait_Ack();
    EEPRom_TXD_Byte(((ReadAddr - 1) * len) & 0xff);   //发送低地址
    EEPRom_Wait_Ack();
    EEPRom_IIC_Start();
    EEPRom_TXD_Byte(EEPRom_ReadCMD + (DeviceAddr << 1));           //进入接收模式
    EEPRom_Wait_Ack();
    for(i = 0; i < len; i ++)
    {
        if(i < (len - 1))
        {
            DataToRead[i] = EEPRom_RXD_Byte(1);//不应答
        }
        else
        {
            DataToRead[i] = EEPRom_RXD_Byte(0);//应答
        }
        DataToCompare[i] = *(DataToRead + i);
    }
    EEPRom_IIC_Stop();//产生一个停止条件
    if (!DeviceAddr)
    {
        if (Err_times >= 0)
        {
            if (CRC_16(EE_Compare,len) == CRC_16(DataToCompare,len))
            {
                delay_us(100);
            }
            else
            {
                for(i = 0; i < len; i ++)
                {
                    EE_Compare[i] = DataToCompare[i];
                }
                Err_times--;
                return EEPRom_ReadDataLen(DeviceAddr,ReadAddr,DataToRead,len);
            }
        }
        else
        {
            Error_Record = 1;
            return 0;
        }
        Err_times = EE_CRC_TIMES;
    }
    return 0;
//    }
}
//*******************************************
//该函数用于写入32bit的数据
//WriteAddr  :开始写入字节的首地址  地址a(n) = a(1) + (n - 1) * d
//DataToWrite:数据
void EEPRom_WriteData(u16 WriteAddr,u32 DataToWrite)
{
    u8 t;
    if(WriteAddr < 2048)//写入第一个外部EE
    {
        for(t = 0; t < 4; t ++)
        {
            EEPRom_WriteOneByte(0,(1 + (WriteAddr - 1) * 4) + t,(((DataToWrite << (8 * t)) & 0xff000000) >> 24) & 0xff);
        }
    }
    else//写入第二个外部EE
    {
        for(t = 0; t < 4; t ++)
        {
            EEPRom_WriteOneByte(1,WriteAddr + t,(((DataToWrite << (8 * t)) & 0xff000000) >> 24) & 0xff);
        }
    }
}
//*******************************************
//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 EEPRom_ReadData(u16 ReadAddr,u8 Len)
{
    u8 t;
    u32 temp=0;
    for(t = 0; t < Len; t ++)
    {
        temp <<= 8;
        temp += EEPRom_ReadOneByte(0,ReadAddr + Len - t - 1);
    }
    return temp;
}
