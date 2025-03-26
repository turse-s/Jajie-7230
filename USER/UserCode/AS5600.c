#include "AS5600.h"
#include "delay.h"
//#include "includes.h"
/*********************************************************************************
*磁编码器AS5600
********************************************************************************/
u8 addr=0x36;
u8 Motor_Direction = 0;
//extern u8 Motor_Direction;
//extern u8 Motor_Direction_T;
void Pan_SDA_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = Pan_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_I2C_SDA_GPIO, &GPIO_InitStructure);//初始化
}
void Pan_SDA_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = Pan_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_I2C_SDA_GPIO, &GPIO_InitStructure);//初始化
}
//初始化IIC
void Pan_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(Pan_I2C_SCL_RCC | Pan_I2C_SDA_RCC, ENABLE);//使能GPIOB时钟
//  GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE);//释放JTAG引脚
    GPIO_InitStructure.GPIO_Pin = Pan_I2C_SCL_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_I2C_SCL_GPIO,&GPIO_InitStructure);//初始化
    GPIO_InitStructure.GPIO_Pin = Pan_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_I2C_SDA_GPIO,&GPIO_InitStructure);//初始化
    Pan_IIC_SCL = 1;
    Pan_IIC_SDA = 1;
}
//产生IIC起始信号
void Pan_IIC_Start(void)
{
    Pan_SDA_OUT();     //sda线输出
    Pan_IIC_SDA=1;
    Pan_IIC_SCL=1;
    delay_us(4);
    Pan_IIC_SDA=0;//START:when CLK is high,DATA change form high to low
    delay_us(4);
    Pan_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void Pan_IIC_Stop(void)
{
    Pan_SDA_OUT();//sda线输出
    Pan_IIC_SCL=0;
    Pan_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
    delay_us(4);
    Pan_IIC_SCL=1;
    Pan_IIC_SDA=1;//发送I2C总线结束信号
    delay_us(4);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 Pan_IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    Pan_SDA_IN();      //SDA设置为输入
    Pan_IIC_SDA=1;
    delay_us(1);
    Pan_IIC_SCL=1;
    delay_us(1);
    while(Pan_READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            Pan_IIC_Stop();
            return 1;
        }
    }
    Pan_IIC_SCL=0;//时钟输出0
    return 0;
}
//产生ACK应答
void Pan_IIC_Ack(void)
{
    Pan_IIC_SCL=0;
    Pan_SDA_OUT();
    Pan_IIC_SDA=0;
    delay_us(2);
    Pan_IIC_SCL=1;
    delay_us(2);
    Pan_IIC_SCL=0;
}
//不产生ACK应答
void Pan_IIC_NAck(void)
{
    Pan_IIC_SCL=0;
    Pan_SDA_OUT();
    Pan_IIC_SDA=1;
    delay_us(2);
    Pan_IIC_SCL=1;
    delay_us(2);
    Pan_IIC_SCL=0;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void Pan_IIC_Send_Byte(u8 txd)
{
    u8 t;
    Pan_SDA_OUT();
    Pan_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0; t<8; t++)
    {
        //IIC_SDA=(txd&0x80)>>7;
        if((txd&0x80)>>7)
        {
            Pan_IIC_SDA=1;
        }
        else
        {
            Pan_IIC_SDA=0;
        }
        txd<<=1;
        delay_us(2);   //对TEA5767这三个延时都是必须的
        Pan_IIC_SCL=1;
        delay_us(2);
        Pan_IIC_SCL=0;
        delay_us(2);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 Pan_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    Pan_SDA_IN();//SDA设置为输入
    for(i=0; i<8; i++ )
    {
        Pan_IIC_SCL=0;
        delay_us(2);
        Pan_IIC_SCL=1;
        receive<<=1;
        if(Pan_READ_SDA)
        {
            receive++;
        }
        delay_us(1);
    }
    if (!ack)
    {
        Pan_IIC_NAck();    //发送nACK
    }
    else
    {
        Pan_IIC_Ack();    //发送ACK
    }
    return receive;
}
void Tilt_SDA_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = Tilt_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_I2C_SDA_GPIO, &GPIO_InitStructure);//初始化
}
void Tilt_SDA_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = Tilt_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_I2C_SDA_GPIO, &GPIO_InitStructure);//初始化
}
//初始化IIC
void Tilt_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(Tilt_I2C_SCL_RCC | Tilt_I2C_SDA_RCC, ENABLE);//使能GPIOB时钟
    GPIO_InitStructure.GPIO_Pin = Tilt_I2C_SCL_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_I2C_SCL_GPIO, &GPIO_InitStructure);//初始化
    GPIO_InitStructure.GPIO_Pin = Tilt_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_I2C_SDA_GPIO, &GPIO_InitStructure);//初始化
    Tilt_IIC_SCL = 1;
    Tilt_IIC_SDA = 1;
    //GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE);//释放JTAG引脚
}
//产生IIC起始信号
void Tilt_IIC_Start(void)
{
    Tilt_SDA_OUT();     //sda线输出
    Tilt_IIC_SDA=1;
    Tilt_IIC_SCL=1;
    delay_us(4);
    Tilt_IIC_SDA=0;//START:when CLK is high,DATA change form high to low
    delay_us(4);
    Tilt_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void Tilt_IIC_Stop(void)
{
    Tilt_SDA_OUT();//sda线输出
    Tilt_IIC_SCL=0;
    Tilt_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
    delay_us(4);
    Tilt_IIC_SCL=1;
    Tilt_IIC_SDA=1;//发送I2C总线结束信号
    delay_us(4);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 Tilt_IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    Tilt_SDA_IN();      //SDA设置为输入
    Tilt_IIC_SDA=1;
    delay_us(1);
    Tilt_IIC_SCL=1;
    delay_us(1);
    while(Tilt_READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            Tilt_IIC_Stop();
            return 1;
        }
    }
    Tilt_IIC_SCL=0;//时钟输出0
    return 0;
}
//产生ACK应答
void Tilt_IIC_Ack(void)
{
    Tilt_IIC_SCL=0;
    Tilt_SDA_OUT();
    Tilt_IIC_SDA=0;
    delay_us(2);
    Tilt_IIC_SCL=1;
    delay_us(2);
    Tilt_IIC_SCL=0;
}
//不产生ACK应答
void Tilt_IIC_NAck(void)
{
    Tilt_IIC_SCL=0;
    Tilt_SDA_OUT();
    Tilt_IIC_SDA=1;
    delay_us(2);
    Tilt_IIC_SCL=1;
    delay_us(2);
    Tilt_IIC_SCL=0;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void Tilt_IIC_Send_Byte(u8 txd)
{
    u8 t;
    Tilt_SDA_OUT();
    Tilt_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0; t<8; t++)
    {
        //IIC_SDA=(txd&0x80)>>7;
        if((txd&0x80)>>7)
        {
            Tilt_IIC_SDA=1;
        }
        else
        {
            Tilt_IIC_SDA=0;
        }
        txd<<=1;
        delay_us(2);   //对TEA5767这三个延时都是必须的
        Tilt_IIC_SCL=1;
        delay_us(2);
        Tilt_IIC_SCL=0;
        delay_us(2);
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 Tilt_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    Tilt_SDA_IN();//SDA设置为输入
    for(i=0; i<8; i++ )
    {
        Tilt_IIC_SCL=0;
        delay_us(2);
        Tilt_IIC_SCL=1;
        receive<<=1;
        if(Tilt_READ_SDA)
        {
            receive++;
        }
        delay_us(1);
    }
    if(!ack)
    {
        Tilt_IIC_NAck();    //发送nACK
    }
    else
    {
        Tilt_IIC_Ack();    //发送ACK
    }
    return receive;
}
u16 Pan_get_Angle()
{
    u32 code=0;
    u8 status=0;
    /***读高四位***/
    Pan_IIC_Start();
    Pan_IIC_Send_Byte(0x6c);
    Pan_IIC_Wait_Ack();     //等待应答
    Pan_IIC_Send_Byte(0x0C);
    Pan_IIC_Wait_Ack();     //等待应答
    Pan_IIC_Start();
    Pan_IIC_Send_Byte(0x6d);
    Pan_IIC_Wait_Ack();     //等待应答
    code=Pan_IIC_Read_Byte(0);
    Pan_IIC_Stop();
    delay_us(2);
    /*读低8位*/
    Pan_IIC_Start();
    Pan_IIC_Send_Byte(0x6c);
    Pan_IIC_Wait_Ack();     //等待应答
    Pan_IIC_Send_Byte(0x0D);
    Pan_IIC_Wait_Ack();     //等待应答
    Pan_IIC_Start();
    Pan_IIC_Send_Byte(0x6d);
    Pan_IIC_Wait_Ack();     //等待应答
    //code=IIC_Read_Byte(0);
    code=(code<<8)|(Pan_IIC_Read_Byte(0));
    Pan_IIC_Stop();
    delay_us(2);
    /*读状态*/
    Pan_IIC_Start();
    Pan_IIC_Send_Byte(0x6c);
    Pan_IIC_Wait_Ack();     //等待应答
    Pan_IIC_Send_Byte(0x0B);
    Pan_IIC_Wait_Ack();     //等待应答
    Pan_IIC_Start();
    Pan_IIC_Send_Byte(0x6d);
    Pan_IIC_Wait_Ack();     //等待应答
    status=Pan_IIC_Read_Byte(0);
    Pan_IIC_Stop();
    if((code << 8) + status != 0xffffff)
    {
        if((status & 0x38)==0x20)     //编码器状态正确
        {
            if(Motor_Direction&0x80)
            {
                code = code & 0x0fff;
            }
            else
            {
                code = (4096- code & 0x0fff);
            }
            Pan_Encoder_State = Encoder_Normal;
        }
        else
        {
            code=0xEEEE;
            Pan_Encoder_State = Encoder_Error;
        }
    }
    else
    {
        code=0xEEEE;
        Pan_Encoder_State = Encoder_Drop;
    }
    return code;
}
u8 app = 0;
u16 Tilt_get_Angle()
{
    u16 code=0;
    u8 status=0;
    /*读高4位*/
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6c);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Send_Byte(0x0C);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6d);
    Tilt_IIC_Wait_Ack();        //等待应答
    code=Tilt_IIC_Read_Byte(0);
    Tilt_IIC_Stop();
    delay_us(2);
    /*读低8位*/
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6c);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Send_Byte(0x0D);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6d);
    Tilt_IIC_Wait_Ack();        //等待应答
    //code=IIC_Read_Byte(0);
    code=(code<<8)|(Tilt_IIC_Read_Byte(0));
    Tilt_IIC_Stop();
    delay_us(2);
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6c);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Send_Byte(0x0B);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6d);
    Tilt_IIC_Wait_Ack();        //等待应答
    status=Tilt_IIC_Read_Byte(0);
    Tilt_IIC_Stop();
    if((code << 8) + status != 0xffffff)
    {
        if((status & 0x38)==0x20)     //编码器状态正确
        {
            //把方位调正了，齿轮有是反的
            if(Motor_Direction&0x80)
            {
                code = code & 0x0fff;
            }
            else
            {
                code = (4096- code & 0x0fff);
            }
            Tilt_Encoder_State = Encoder_Normal;
        }
        else
        {
            code=0xEEEE;
            Tilt_Encoder_State = Encoder_Error;
        }
    }
    else
    {
        code=0xEEEE;
        Tilt_Encoder_State = Encoder_Drop;
    }
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6c);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Send_Byte(0x1a);
    Tilt_IIC_Wait_Ack();        //等待应答
    Tilt_IIC_Start();
    Tilt_IIC_Send_Byte(0x6d);
    Tilt_IIC_Wait_Ack();        //等待应答
    app=Tilt_IIC_Read_Byte(0);
    Tilt_IIC_Stop();
    delay_us(2);
    return code;
}
