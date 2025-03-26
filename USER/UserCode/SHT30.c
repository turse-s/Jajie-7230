#include "SHT30.h"
#include "delay.h"
#include "includes.h"
//static struct rt_i2c_bus_device *i2c_bus = NULL;     /* I2C总线设备句柄 */
//static rt_bool_t initialized = RT_FALSE;                /* 传感器初始化状态 */
///* 写传感器寄存器 */
//rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t reg, rt_uint8_t *data)
//{
//    rt_uint8_t buf[2];
//    struct rt_i2c_msg msgs;
//    buf[0] = reg; //cmd
//    buf[1] = data[0];
//    msgs.addr = SHT30_ADDR;
//    msgs.flags = RT_I2C_WR;
//    msgs.buf = buf;
//    msgs.len = 2;
//    /* 调用I2C设备接口传输数据 */
//    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
//    {
//        return RT_EOK;
//    }
//    else
//    {
//        return -RT_ERROR;
//    }
//}
///* 读传感器寄存器数据 */
//rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t len, rt_uint8_t *buf)
//{
//    struct rt_i2c_msg msgs;
//    msgs.addr = SHT30_ADDR;
//    msgs.flags = RT_I2C_RD;
//    msgs.buf = buf;
//    msgs.len = len;
//    /* 调用I2C设备接口传输数据 */
//    if (rt_i2c_transfer(bus, &msgs, 1) == 1)
//    {
//        return RT_EOK;
//    }
//    else
//    {
//        return -RT_ERROR;
//    }
//}
//void read_temp_humi(float *cur_temp, float *cur_humi)
//{
//    uint8_t temp[6];
//    write_reg(i2c_bus,0xe0,0x00);      /* 发送命令 */
//    rt_thread_mdelay(400);
//    read_regs(i2c_bus,6,temp);                /* 获取传感器数据 */
//    /* 湿度数据转换 */
//    *cur_humi = (temp[1] << 12 | temp[2] << 4 | (temp[3] & 0xf0) >> 4) * 100.0 / (1 << 20);
//    /* 温度数据转换 */
//    *cur_temp = ((temp[3] & 0xf) << 16 | temp[4] << 8 | temp[5]) * 200.0 / (1 << 20) - 50;
//}
//void SHT30_init(const char *name)
//{
//    uint8_t temp[2] = {0, 0};
//    /* 查找I2C总线设备，获取I2C总线设备句柄 */
//    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(name);
//    if (i2c_bus == RT_NULL)
//    {
//        rt_kprintf("can't find %s device!\n", name);
//    }
//    else
//    {
//        temp[0] = 0x30;
//        write_reg(i2c_bus, 0x21,temp);
//        rt_thread_mdelay(400);
//        initialized = RT_TRUE;
//    }
//}
//void i2c_SHT30_sample(void)
//{
//    float humidity,temperature;
//    char name[RT_NAME_MAX];
//    humidity = 0.0;
//    temperature = 0.0;
//    rt_strncpy(name,SHT30_I2C_BUS_NAME,RT_NAME_MAX);
//    if (!initialized)
//    {
//        /* 传感器初始化 */
//        SHT30_init(name);
//    }
//    if (initialized)
//    {
//        /* 读取温湿度数据 */
//        read_temp_humi(&temperature, &humidity);
//        rt_kprintf("read SHT30 sensor humidity   : %d.%d %%\n", (int)humidity, (int)(humidity * 10) % 10);
//        if(temperature >= 0)
//        {
//            rt_kprintf("read SHT30 sensor temperature: %d.%d°C\n", (int)temperature, (int)(temperature * 10) % 10);
//        }
//        else
//        {
//            rt_kprintf("read SHT30 sensor temperature: %d.%d°C\n", (int)temperature, (int)(-temperature * 10) % 10);
//        }
//    }
//    else
//    {
//        rt_kprintf("initialize sensor failed!\n");
//    }
//}
//*****************************
//初始化EEPRom
void SHT30_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(SHT30_RCC,ENABLE);
    //配置管脚状态
    GPIO_InitStructure.GPIO_Pin = SHT30_SDA_Pin | SHT30_SCL_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(SHT30_GPIO,&GPIO_InitStructure);
    SHT30_SDA_H;
    SHT30_SCL_H;
}
//*****************************
//设置SHT30_SDA方向,iotype 0:输出，1：输入
void SHT30_SDA_IO_SET(u8 iotype)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(SHT30_SDA_RCC,ENABLE);
    switch(iotype)
    {
    case 0:
    {
        GPIO_InitStructure.GPIO_Pin = SHT30_SDA_Pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    }
    break;
    case 1:
    {
        GPIO_InitStructure.GPIO_Pin = SHT30_SDA_Pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    }
    break;
    }
    GPIO_Init(SHT30_SDA_GPIO,&GPIO_InitStructure);
}
//*****************************
//IIC起始信号
void SHT30_IIC_Start(void)
{
    SHT30_SDA_OUT;
    SHT30_SDA_H;
    SHT30_SCL_H;
    delay_us(4);
    SHT30_SDA_L;
    delay_us(4);
    SHT30_SCL_L;
}
//*****************************
//IIC停止信号
void SHT30_IIC_Stop(void)
{
    SHT30_SDA_OUT;
    SHT30_SCL_L;
    SHT30_SDA_L;
    SHT30_SCL_H;
    delay_us(4);
    SHT30_SDA_H;
    delay_us(4);
}
//*******************************************
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 SHT30_Wait_Ack(void)
{
    u8 errtime = 0;
    SHT30_SDA_IN;
    SHT30_SDA_H;
    delay_us(1);
    SHT30_SCL_H;
    delay_us(1);
    while(SHT30_Read_SDA)
    {
        errtime ++;
        if(errtime > 250)
        {
            SHT30_IIC_Stop();
            return 1;
        }
    }
    SHT30_SCL_L;
    delay_us(1);
    return 0;
}
//*******************************************
//产生应答
void SHT30_Ack(void)
{
    SHT30_SCL_L;
    SHT30_SDA_OUT;
    SHT30_SDA_L;
    delay_us(2);
    SHT30_SCL_H;
    delay_us(2);
    SHT30_SCL_L;
}
//*******************************************
//产生应答
void SHT30_nAck(void)
{
    SHT30_SCL_L;
    SHT30_SDA_OUT;
    SHT30_SDA_H;
    delay_us(2);
    SHT30_SCL_H;
    delay_us(2);
    SHT30_SCL_L;
}
//*****************************
//发送字节
void SHT30_TXD_Byte(u8 data)
{
    u8 i;
    SHT30_SDA_OUT;
    SHT30_SCL_L;
//    delay_us(2);
    for(i = 0; i < 8; i ++)
    {
        delay_us(2);
        if(data & 0x80)
        {
            SHT30_SDA_H;
        }
        else
        {
            SHT30_SDA_L;
        }
        data <<= 1;
        delay_us(2);
        SHT30_SCL_H;
        delay_us(2);
        SHT30_SCL_L;
        delay_us(2);
    }
}
//*****************************
//读取字节
u8 SHT30_RXD_Byte(u8 ack)
{
    u8 i,receive;
    SHT30_SDA_IN;
//    delay_us(2);
    for(i = 0; i < 8; i ++)
    {
        SHT30_SCL_L;
        delay_us(2);
        SHT30_SCL_H;
        receive <<= 1;
        if(SHT30_Read_SDA)
        {
            receive ++;
        }
        delay_us(2);
    }
    if(ack)
    {
        SHT30_Ack();
    }
    else
    {
        SHT30_nAck();
    }
    return receive;
}
//*******************************************
void SHT30_WriteData(u16 WriteAddr,u16 Command)
{
    CPU_SR_ALLOC();
    OS_CRITICAL_ENTER();//进入临界区
    SHT30_IIC_Start();
    SHT30_TXD_Byte(WriteAddr + SHT30_WriteCMD);
    SHT30_Wait_Ack();
    SHT30_TXD_Byte((Command >> 8) & 0xff);//发送命令高字节
    SHT30_Wait_Ack();
    SHT30_TXD_Byte(Command & 0xff);//发送命令低字节
    SHT30_Wait_Ack();
    SHT30_IIC_Stop();//产生一个停止条件
    delay_us(10000);
    OS_CRITICAL_EXIT(); //退出临界区
}
//*******************************************
void SHT30_ReadData(u16 ReadAddr,u16 Command,u8 *data,u8 len)
{
    u8 i = 0;
    CPU_SR_ALLOC();
    OS_CRITICAL_ENTER();//进入临界区
    SHT30_IIC_Start();
    SHT30_TXD_Byte(ReadAddr + SHT30_WriteCMD);
    SHT30_Wait_Ack();
    SHT30_TXD_Byte((Command >> 8) & 0xff);//发送命令高字节
    SHT30_Wait_Ack();
    SHT30_TXD_Byte(Command & 0xff);//发送命令低字节
    SHT30_Wait_Ack();
    SHT30_IIC_Start();
    SHT30_TXD_Byte(ReadAddr + SHT30_ReadCMD);//进入接收模式
    SHT30_Wait_Ack();
    for(i = 0; i < (len - 1); i ++)
    {
        data[i] = SHT30_RXD_Byte(1);
    }
    data[len - 1] = SHT30_RXD_Byte(0);//
    SHT30_IIC_Stop();//产生一个停止条件
    OS_CRITICAL_EXIT(); //退出临界区
}
