#include "hardware.h"
#include "parameters.h"
#ifndef FOC

GPIO_port_t LED1_port =
{
    .GPIOx = GPIOC,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOC,
    .GPIO_Pin_x = GPIO_Pin_3,
};
GPIO_port_t LED2_port =
{
    .GPIOx = GPIOC,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOC,
    .GPIO_Pin_x = GPIO_Pin_2,
};
#endif
//GPIO_port_t Optocoupler_port =
//{
//    .GPIOx = GPIOB,
//    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOB,
//    .GPIO_Pin_x = GPIO_Pin_7,
//};
//GPIO_port_t Relay_port =
//{
//    .GPIOx = GPIOB,
//    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOB,
//    .GPIO_Pin_x = GPIO_Pin_6,
//};
#ifdef PT_LOAC_2215
#ifdef FOC
SPIx_port_t Pan_SPI_port =
{
    .GPIOx = GPIOB,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOB,
    .SPI_NSS_Pin = GPIO_Pin_12,
    .SPI_SCK_Pin = GPIO_Pin_13,
    .SPI_MISO_Pin = GPIO_Pin_14,
    .SPI_MOSI_Pin = GPIO_Pin_15,
};
SPIx_port_t Tilt_SPI_port =
{
    .GPIOx = GPIOC,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOC,
    .SPI_NSS_Pin = GPIO_Pin_15,
    .SPI_SCK_Pin = GPIO_Pin_10,
    .SPI_MISO_Pin = GPIO_Pin_11,
    .SPI_MOSI_Pin = GPIO_Pin_12,
};
#else
SPIx_port_t Pan_SPI_port =
{
    .GPIOx = GPIOA,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOA,
    .SPI_NSS_Pin = GPIO_Pin_4,
    .SPI_SCK_Pin = GPIO_Pin_5,
    .SPI_MISO_Pin = GPIO_Pin_6,
    .SPI_MOSI_Pin = GPIO_Pin_7,
};
SPIx_port_t Tilt_SPI_port =
{
    .GPIOx = GPIOB,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOB,
    .SPI_NSS_Pin = GPIO_Pin_12,
    .SPI_SCK_Pin = GPIO_Pin_13,
    .SPI_MISO_Pin = GPIO_Pin_14,
    .SPI_MOSI_Pin = GPIO_Pin_15,
};
#endif
#else
SPIx_port_t Tilt_SPI_port =
{
    .GPIOx = GPIOA,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOA,
    .SPI_NSS_Pin = GPIO_Pin_4,
    .SPI_SCK_Pin = GPIO_Pin_5,
    .SPI_MISO_Pin = GPIO_Pin_6,
    .SPI_MOSI_Pin = GPIO_Pin_7,
};
SPIx_port_t Pan_SPI_port =
{
    .GPIOx = GPIOB,
    .RCC_AHB1Periph_GPIOx = RCC_AHB1Periph_GPIOB,
    .SPI_NSS_Pin = GPIO_Pin_12,
    .SPI_SCK_Pin = GPIO_Pin_13,
    .SPI_MISO_Pin = GPIO_Pin_14,
    .SPI_MOSI_Pin = GPIO_Pin_15,
};
#endif
//***********************************************************************************
//
//                                  Configure/Initialize
//
//***********************************************************************************
//鐢垫簮鎺夌數涓柇
void PWR_PVD_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//使能PWR时钟
    NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;           //使能PVD所在的外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;          //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);
    EXTI_StructInit(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line16;             //PVD连接到中断线16上
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     //使用中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //电压低于阀值时产生中断
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;               //使能中断线
    EXTI_Init(&EXTI_InitStructure);                         //初始
//    PWR_PVDLevelConfig(PWR_PVDLevel_6);//设定监控阀值   注意和F1的区别  F1的电压等级就更直观   PWR_PVDLevel_2V8
    PWR_PVDLevelConfig(PWR_PVDLevel_7);
    PWR_PVDCmd(ENABLE);//使能PVD
}
void IWDG_Init(u8 psc,u16 rld)      //IWDG是在IAP中初始化并开启的，APP中只管喂狗
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);//
    IWDG_SetPrescaler(psc); //设置IWDG分频系数
    IWDG_SetReload(rld);   //设置IWDG装载值
    IWDG_ReloadCounter(); //reload
    IWDG_Enable();       //使能看门狗
}
void GET_ID_GPIO_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(ID_LOAD_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(ID_CLK_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(ID_QH_RCC,ENABLE);
    GPIO_InitStructure.GPIO_Pin = ID_LOAD_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ID_LOAD_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ID_CLK_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ID_CLK_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ID_QH_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ID_QH_GPIO,&GPIO_InitStructure);
    ID_LOAD = 0;
//    ID_CLK = 1;
//    ID_QH = 1;
}
#ifdef LASER_SETUP
//Laser_Init
void Laser_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(LDR_RCC,ENABLE);//
    GPIO_InitStructure.GPIO_Pin = LDR_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//
    GPIO_Init(LDR_GPIO,&GPIO_InitStructure);
}
#endif
//void TIM2_Init(u16 arr,u16 psc)
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  TIM_DeInit(TIM2);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
//  TIM_TimeBaseStructure.TIM_Period = arr;
//  TIM_TimeBaseStructure.TIM_Prescaler =psc;
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//  TIM_Cmd(TIM2,ENABLE);
////    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //能产生TIME2中断
////    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
////    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级只有1个等级
////    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //使能中断
////    NVIC_Init(&NVIC_InitStructure);
//}
void TIM6_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    TIM_DeInit(TIM6);
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler =psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM6,TIM_FLAG_Update);
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM6,ENABLE);
}
void TIM7_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    TIM_DeInit(TIM7);
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler =psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM7,TIM_FLAG_Update);
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM7,ENABLE);
}
void TIM10_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
    TIM_DeInit(TIM10);
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler =psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM10,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM10,TIM_FLAG_Update);
    TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM10,ENABLE);
}
//GPIO_PORT_INIT
void GPIO_PORT_INIT(GPIO_port_t pHandle)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(pHandle.RCC_AHB1Periph_GPIOx, ENABLE);//使能GPIOB时钟
    GPIO_InitStructure.GPIO_Pin = pHandle.GPIO_Pin_x;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
    GPIO_Init(pHandle.GPIOx,&GPIO_InitStructure);
}
//void GPIO_PORT_INIT_IN(GPIO_port_t pHandle)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_AHB1PeriphClockCmd(pHandle.RCC_AHB1Periph_GPIOx, ENABLE);//使能GPIOB时钟
//    GPIO_InitStructure.GPIO_Pin = pHandle.GPIO_Pin_x;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
//    GPIO_Init(pHandle.GPIOx,&GPIO_InitStructure);
//}
//SPI_PORT_INIT
void SPI_PORT_INIT(SPIx_port_t pHandle)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(pHandle.RCC_AHB1Periph_GPIOx,ENABLE);
    GPIO_InitStructure.GPIO_Pin = pHandle.SPI_NSS_Pin | pHandle.SPI_SCK_Pin | pHandle.SPI_MOSI_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(pHandle.GPIOx,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pHandle.SPI_MISO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(pHandle.GPIOx,&GPIO_InitStructure);
    GPIO_SetBits(pHandle.GPIOx,pHandle.SPI_NSS_Pin);
    GPIO_SetBits(pHandle.GPIOx,pHandle.SPI_SCK_Pin);
}
#ifdef FOC
void SPI_PORT_INIT_Tilt(SPIx_port_t pHandle)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(pHandle.RCC_AHB1Periph_GPIOx,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    GPIO_InitStructure.GPIO_Pin = pHandle.SPI_SCK_Pin | pHandle.SPI_MOSI_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(pHandle.GPIOx,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pHandle.SPI_MISO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(pHandle.GPIOx,&GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = pHandle.SPI_NSS_Pin;//PA15
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    GPIO_SetBits(GPIOA,pHandle.SPI_NSS_Pin);
    GPIO_SetBits(pHandle.GPIOx,pHandle.SPI_SCK_Pin);
}

void SPIx_Init(SPI_TypeDef *SPIx,GPIO_TypeDef *SPI_PORT,uint16_t Pin_CLK,uint16_t Pin_MISO,uint16_t Pin_MOSI,GPIO_TypeDef *CS_PORT,uint16_t Pin_CS,u8 AF_pinCLK,u8 AF_pinMISO,u8 AF_pinMOSI)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	if(SPIx==SPI1)
		{
			 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟
		}
	if(SPIx==SPI2)
	{
		 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI1时钟
	}
	if(SPIx==SPI3)
	{
		 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//使能SPI1时钟
	}
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOB时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOB时钟
	
	GPIO_InitStructure.GPIO_Pin = Pin_CS;//cs
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(CS_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(CS_PORT,Pin_CS);// 拉高片选
 
  //GPIOFB3,4,5初始化设置
  GPIO_InitStructure.GPIO_Pin =Pin_CLK|Pin_MISO|Pin_MOSI;//PB3~5复用功能输出	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(SPI_PORT, &GPIO_InitStructure);//初始化

		if(SPIx==SPI1)
		{
			GPIO_PinAFConfig(SPI_PORT,AF_pinCLK,GPIO_AF_SPI1); //PB3复用为 SPI1
			GPIO_PinAFConfig(SPI_PORT,AF_pinMISO,GPIO_AF_SPI1); //PB4复用为 SPI1
			GPIO_PinAFConfig(SPI_PORT,AF_pinMOSI,GPIO_AF_SPI1); //PB5复用为 SPI1
			//这里只针对SPI口初始化
			RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
			RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1
		}
		if(SPIx==SPI2)
		{
			GPIO_PinAFConfig(SPI_PORT,AF_pinCLK,GPIO_AF_SPI2); //PB3复用为 SPI1
			GPIO_PinAFConfig(SPI_PORT,AF_pinMISO,GPIO_AF_SPI2); //PB4复用为 SPI1
			GPIO_PinAFConfig(SPI_PORT,AF_pinMOSI,GPIO_AF_SPI2); //PB5复用为 SPI1
			//这里只针对SPI口初始化
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI1
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI1
		}
		if(SPIx==SPI3)
		{
			GPIO_PinAFConfig(SPI_PORT,AF_pinCLK,GPIO_AF_SPI3); //PB3复用为 SPI1
			GPIO_PinAFConfig(SPI_PORT,AF_pinMISO,GPIO_AF_SPI3); //PB4复用为 SPI1
			GPIO_PinAFConfig(SPI_PORT,AF_pinMOSI,GPIO_AF_SPI3); //PB5复用为 SPI1
			//这里只针对SPI口初始化
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//复位SPI1
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1
		}

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
		
	SPI_Init(SPIx, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPIx, ENABLE); //使能SPI外设

//	SPI1_ReadWriteByte(0xff);//启动传输		 
} 
#endif
//EncoderPortInitialize
void Encoder_Pan_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_AHB1PeriphClockCmd(Pan_SSI_CS_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Pan_SSI_CK_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Pan_SSI_DO_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Pan_Index_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Pan_Dir_RCC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource2);//
    GPIO_InitStructure.GPIO_Pin = Pan_SSI_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_SSI_CS_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Pan_SSI_CK_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_SSI_CK_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Pan_SSI_DI_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_SSI_DI_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Pan_SSI_DO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_SSI_DO_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Pan_Index_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Pan_Index_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Pan_Dir_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
    GPIO_Init(Pan_Dir_GPIO,&GPIO_InitStructure);
//    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//
//    EXTI_ClearFlag(EXTI_Line2);
//    EXTI_ClearITPendingBit(EXTI_Line2);//
    Encoder_Pan_EXTI_Line2(DISABLE);
    Pan_SSI_CS_H;
    Pan_SSI_CK_H;
    Pan_SSI_DI_H;
    Pan_SSI_DO_H;
}
void Encoder_Pan_EXTI_Line2(FunctionalState state)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = state;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearFlag(EXTI_Line2);
    EXTI_ClearITPendingBit(EXTI_Line2);//
}
void Encoder_Tilt_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
//    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_AHB1PeriphClockCmd(Tilt_SSI_CS_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Tilt_SSI_CK_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Tilt_SSI_DO_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Tilt_Index_RCC,ENABLE);
//    RCC_AHB1PeriphClockCmd(Tilt_Dir_RCC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource15);//
    GPIO_InitStructure.GPIO_Pin = Tilt_SSI_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_SSI_CS_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Tilt_SSI_CK_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_SSI_CK_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Tilt_SSI_DI_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_SSI_DI_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Tilt_SSI_DO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_SSI_DO_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Tilt_Index_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(Tilt_Index_GPIO,&GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin = Tilt_Dir_Pin;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//输入模式
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
//    GPIO_Init(Tilt_Dir_GPIO,&GPIO_InitStructure);
//    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init(&EXTI_InitStructure);
//
//    EXTI_ClearFlag(EXTI_Line15);
//    EXTI_ClearITPendingBit(EXTI_Line15);//
    Encoder_Tilt_EXTI_Line15(DISABLE);
    Tilt_SSI_CS_H;
    Tilt_SSI_CK_L;
    Tilt_SSI_DI_H;
    Tilt_SSI_DO_H;
}
void Encoder_Tilt_EXTI_Line15(FunctionalState state)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = state;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearFlag(EXTI_Line15);
    EXTI_ClearITPendingBit(EXTI_Line15);//
}
//MotorPortInitialize
//********************************************************************************************
//水平光管检测
void Pan_Calibration_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_AHB1PeriphClockCmd(Pan_Calibration1_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Pan_Calibration2_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Pan_Calibration3_RCC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource5);//
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource6);//
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource7);//
    GPIO_InitStructure.GPIO_Pin = Pan_Calibration1_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(Pan_Calibration1_GPIO,&GPIO_InitStructure);
//
    GPIO_InitStructure.GPIO_Pin = Pan_Calibration2_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(Pan_Calibration2_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Pan_Calibration3_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(Pan_Calibration3_GPIO,&GPIO_InitStructure);
//    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Line = EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
//    EXTI_ClearFlag(EXTI_Line5);
//    EXTI_ClearITPendingBit(EXTI_Line5);//
//    EXTI_ClearFlag(EXTI_Line6);
//    EXTI_ClearITPendingBit(EXTI_Line6);//
    EXTI_ClearFlag(EXTI_Line7);
    EXTI_ClearITPendingBit(EXTI_Line7);//
}
void Pan_Calibration_EXTI_DISABLE(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearFlag(EXTI_Line5);
    EXTI_ClearITPendingBit(EXTI_Line5);
}
//垂直光管检测
void Tilt_Calibration_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_AHB1PeriphClockCmd(Tilt_Calibration1_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Tilt_Calibration2_RCC,ENABLE);
    RCC_AHB1PeriphClockCmd(Tilt_Calibration3_RCC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource9);//
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource0);//
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource1);//
    GPIO_InitStructure.GPIO_Pin = Tilt_Calibration1_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(Tilt_Calibration1_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Tilt_Calibration2_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(Tilt_Calibration2_GPIO,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = Tilt_Calibration3_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;        //输入模式
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(Tilt_Calibration3_GPIO,&GPIO_InitStructure);
//    EXTI_InitStructure.EXTI_Line = EXTI_Line9 | EXTI_Line0 | EXTI_Line1;
//    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
//    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
//    EXTI_ClearFlag(EXTI_Line9);
//    EXTI_ClearITPendingBit(EXTI_Line9);//
//    EXTI_ClearFlag(EXTI_Line0);
//    EXTI_ClearITPendingBit(EXTI_Line0);//
    EXTI_ClearFlag(EXTI_Line1);
    EXTI_ClearITPendingBit(EXTI_Line1);//
}
void Tilt_Calibration_EXTI_DISABLE(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    RCC_AHB1PeriphClockCmd(Tilt_Calibration1_RCC,ENABLE);
    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearFlag(EXTI_Line9);
    EXTI_ClearITPendingBit(EXTI_Line9);
}
void Tilt_Calibration_EXTI_ENABLE(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_ClearFlag(EXTI_Line9);
    EXTI_ClearITPendingBit(EXTI_Line9);
}
#ifndef FOC
//********************************************************************************************
//水平速度检测
void TIM5_Init(u16 arr,u16 psc)//正交编码器模式，水平
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd= GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
    TIM_DeInit(TIM5);
    TIM_TimeBaseStructure.TIM_Period = arr * 2;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM5,TIM_EncoderMode_TI1,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);   //
    TIM_ICStructInit(&TIM_ICInitStructure);                                                                                                                                     //
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
    TIM5->CNT = arr;
    TIM_Cmd(TIM5, ENABLE);
}
//垂直速度检测
void TIM4_Init(u16 arr,u16 psc)//正交编码器模式，垂直
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period = arr * 2;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising);    //
    TIM_ICStructInit(&TIM_ICInitStructure);                                                                                                                                     //
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM4->CNT = arr;
    TIM_Cmd(TIM4, ENABLE);
}
//********************************************************************************************
//水平电机端口
void Motor_Pan_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructer;
    RCC_AHB1PeriphClockCmd(Motor_Pan_DIR_RCC | Motor_Pan_BRK_RCC, ENABLE);
    GPIO_InitStructer.GPIO_Pin = Motor_Pan_DIR_Pin;         //马达1的方向
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_OUT;        //普通输出模式
    GPIO_InitStructer.GPIO_OType = GPIO_OType_PP;   //推挽输出
    GPIO_InitStructer.GPIO_PuPd = GPIO_PuPd_UP; //
    GPIO_Init(Motor_Pan_DIR_GPIO, &GPIO_InitStructer);
    GPIO_InitStructer.GPIO_Pin = Motor_Pan_BRK_Pin;         //马达1刹车端口
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_OUT;        //普通输出模式
    GPIO_InitStructer.GPIO_OType = GPIO_OType_PP;   //推挽输出
    GPIO_InitStructer.GPIO_PuPd = GPIO_PuPd_DOWN;   //下拉
    GPIO_Init(Motor_Pan_BRK_GPIO, &GPIO_InitStructer);
    Motor_Pan_BRK(BRK_ON);
}
//垂直电机端口
void Motor_Tilt_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructer;
    RCC_AHB1PeriphClockCmd(Motor_Tilt_DIR_RCC | Motor_Tilt_BRK_RCC, ENABLE);
    GPIO_InitStructer.GPIO_Pin = Motor_Tilt_DIR_Pin;            //马达2的方向端口
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_OUT;        //普通输出模式
    GPIO_InitStructer.GPIO_OType = GPIO_OType_PP;   //推挽输出
    GPIO_InitStructer.GPIO_PuPd = GPIO_PuPd_UP; //
    GPIO_Init(Motor_Tilt_DIR_GPIO, &GPIO_InitStructer);
    GPIO_InitStructer.GPIO_Pin = Motor_Tilt_BRK_Pin;            //马达2刹车端口
    GPIO_InitStructer.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode=GPIO_Mode_OUT;      //普通输出模式
    GPIO_InitStructer.GPIO_OType = GPIO_OType_PP;   //推挽输出
    GPIO_InitStructer.GPIO_PuPd = GPIO_PuPd_DOWN;   //下拉
    GPIO_Init(Motor_Tilt_BRK_GPIO, &GPIO_InitStructer);
    Motor_Tilt_BRK(BRK_ON);
}
void Motor_Pan_PWM_Init(u32 arr,u32 psc)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
    RCC_AHB1PeriphClockCmd(Motor_Pan_PWM_RCC,ENABLE);
    GPIO_PinAFConfig(Motor_Pan_PWM_PinAF);
    GPIO_InitStructure.GPIO_Pin = Motor_Pan_PWM_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(Motor_Pan_PWM_GPIO,&GPIO_InitStructure);
    //初始化定时器9
    TIM_TimeBaseStructure.TIM_Period = arr;                     //设置在下一个更新事件装入活动的自动重装载寄存器周期的值     80K
    TIM_TimeBaseStructure.TIM_Prescaler =psc;                   //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                 //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
    TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    //初始化定时器9 channel PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;        //输出极性:TIM输出比较极性低
    TIM_OC1Init(TIM9,&TIM_OCInitStructure);    //根据T指定的参数初始化外设TIMxOC1
    TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);  //使能TIMx在CCR1上的预装载寄存器
    TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能
    TIM_Cmd(TIM9,ENABLE);  //使能TIMx
//    GPIO_SetBits(Motor_Pan_PWM_GPIO,Motor_Pan_PWM_Pin);
}
void Motor_Tilt_PWM_Init(u32 arr,u32 psc)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
    RCC_AHB1PeriphClockCmd(Motor_Tilt_PWM_RCC,ENABLE);
    GPIO_PinAFConfig(Motor_Tilt_PWM_PinAF);
    GPIO_InitStructure.GPIO_Pin = Motor_Tilt_PWM_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(Motor_Tilt_PWM_GPIO,&GPIO_InitStructure);
    //初始化定时器
    TIM_TimeBaseStructure.TIM_Period = arr;                     //设置在下一个更新事件装入活动的自动重装载寄存器周期的值     80K
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
    TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    //初始化定时器 channel PWM模式
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;               //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;        //输出极性:TIM输出比较极性低
    TIM_OC2Init(TIM9,&TIM_OCInitStructure);    //根据T指定的参数初始化外设TIMxOC2
    TIM_OC2PreloadConfig(TIM9,TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
    TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能
    TIM_Cmd(TIM9,ENABLE);  //使能TIMx
//    GPIO_ResetBits(Motor_Tilt_PWM_GPIO,Motor_Tilt_PWM_Pin);
}
#endif
//********************************************************************************************
/*
DMA_Streamx： 配置的数据流
chx：         DMA通道选择
par ：        外设地址
mar：         存储器地址
ndtr:         数据传输量
*/
void DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,uint32_t DMA_DIR_MP,u16 ndtr)
{
    DMA_InitTypeDef  DMA_InitStructure;
    if((u32)DMA_Streamx>(u32)DMA2)//判断当前stream属于DMA2还是DMA1
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
    }
    else
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
    }
    DMA_DeInit(DMA_Streamx);
    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE) {} //等待DMA可配置（当前数据流EN位为0）
    /* 配置 DMA Stream */
    DMA_InitStructure.DMA_Channel = chx;
    DMA_InitStructure.DMA_PeripheralBaseAddr = par;
    DMA_InitStructure.DMA_Memory0BaseAddr = mar;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MP;//方向：外设存到储器
    DMA_InitStructure.DMA_BufferSize = ndtr;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设指针不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器指针增量模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度：8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度：8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//单次
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器单次突发传输
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设单次突发传输
    DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
    DMA_Cmd(DMA_Streamx,ENABLE);
//        DMA_Cmd(DMA_Streamx,DISABLE);
}
//开启一次DMA传输
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
    DMA_Cmd(DMA_Streamx,DISABLE);                      //先关闭DMA,才能设置它
    while(DMA_GetCmdStatus(DMA_Streamx) != DISABLE) {}  //等待传输结束
    DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //设置传输数据长度
    DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA
}
//USART1PortInitialize
void USART1_INT_Init(u32 bound)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    RCC_AHB1PeriphClockCmd(USART1_RCC,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    GPIO_PinAFConfig(USART1_GPIO,USART1_PinSource_TX,GPIO_AF_USART1);
    GPIO_PinAFConfig(USART1_GPIO,USART1_PinSource_RX,GPIO_AF_USART1);
    GPIO_InitStructure.GPIO_Pin = USART1_Pin_TX | USART1_Pin_RX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(USART1_GPIO, &GPIO_InitStructure);
#ifdef FOC
			//USART1 EN脚配置
		GPIO_InitStructure.GPIO_Pin = USART1_Pin_EN; //GPIOA11
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
		GPIO_Init(USART1_GPIO,&GPIO_InitStructure); //初始化PA11
#endif	
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1,&USART_InitStructure);
    //initialize the DMA channel.
    //******************************************************************
    //USART1    DMA接收
    DMA_Config( USART1_DMA_Steam_RX,
                USART1_DMA_Channel,
                (uint32_t)&(USART1->DR),     //串口DR寄存器
                (uint32_t)DMA_USART1_RX_Buf,//自定义的接收数据buf
                DMA_DIR_PeripheralToMemory,//外设到存储器方向
                DMA_USART1_RX_BUF_SIZE);//长度
    //******************************************************************
    //USART1    DMA发送
    DMA_Config( USART1_DMA_Steam_TX,
                USART1_DMA_Channel,
                (uint32_t)&(USART1->DR),     //串口DR寄存器
                (uint32_t)DMA_USART1_TX_Buf,//自定义的接收数据buf
                DMA_DIR_MemoryToPeripheral,//存储器到外设方向
                0);//长度
    //******************************************************************
    USART1_ENABLE();
}
void USART1_DISABLE(void)
{
    USART_ITConfig(USART1,USART_IT_IDLE,DISABLE);//关闭空闲中断
    USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);//关闭DMA接收
    USART_DMACmd(USART1,USART_DMAReq_Tx,DISABLE);//DMA发送
    USART_Cmd(USART1,DISABLE);
}
void USART1_ENABLE(void)
{
    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//空闲中断
    DMA_ITConfig(USART1_DMA_Steam_TX,DMA_IT_TC,ENABLE);//使能DMA传输完成中断
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//DMA接收
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//DMA发送
    USART_Cmd(USART1,ENABLE);
}
//USART2PortInitialize
void USART2_INT_Init(u32 bound)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    RCC_AHB1PeriphClockCmd(USART2_RCC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    GPIO_PinAFConfig(USART2_TX_PinAF);
    GPIO_PinAFConfig(USART2_RX_PinAF);
    GPIO_InitStructure.GPIO_Pin = USART2_Pin_TX | USART2_Pin_RX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(USART2_GPIO,&GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2,&USART_InitStructure);
    //initialize the DMA channel.
    //******************************************************************
    //USART1    DMA接收
    DMA_Config( USART2_DMA_Steam_RX,
                USART2_DMA_Channel,
                (uint32_t)&(USART2->DR),     //串口DR寄存器
                (uint32_t)DMA_USART2_RX_Buf,//自定义的接收数据buf
                DMA_DIR_PeripheralToMemory,//外设到存储器方向
                DMA_USART2_RX_BUF_SIZE);//长度
    //******************************************************************
    //USART1    DMA发送
    DMA_Config( USART2_DMA_Steam_TX,
                USART2_DMA_Channel,
                (uint32_t)&(USART2->DR),     //串口DR寄存器
                (uint32_t)DMA_USART2_TX_Buf,//自定义的接收数据buf
                DMA_DIR_MemoryToPeripheral,//存储器到外设方向
                0);//长度
    //******************************************************************
    USART2_ENABLE();
}
void USART2_DISABLE(void)
{
    USART_ITConfig(USART2,USART_IT_IDLE,DISABLE);//关闭空闲中断
    USART_DMACmd(USART2,USART_DMAReq_Rx,DISABLE);//关闭DMA接收
    USART_DMACmd(USART2,USART_DMAReq_Tx,DISABLE);//DMA发送
    USART_Cmd(USART2,DISABLE);
}
void USART2_ENABLE(void)
{
    USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);//空闲中断
    DMA_ITConfig(USART2_DMA_Steam_TX,DMA_IT_TC,ENABLE);//使能DMA传输完成中断
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//DMA接收
    USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);//DMA发送
    USART_Cmd(USART2,ENABLE);
}
//USART1PortInitialize
void USART3_INT_Init(u32 bound)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    RCC_AHB1PeriphClockCmd(USART3_RCC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    GPIO_PinAFConfig(USART3_TX_PinAF);
    GPIO_PinAFConfig(USART3_RX_PinAF);
    GPIO_InitStructure.GPIO_Pin = USART3_Pin_TX | USART3_Pin_RX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(USART3_GPIO, &GPIO_InitStructure);
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3,&USART_InitStructure);
//  USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
//    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);//开启空闲中断
//  USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//开启DMA接收
//  USART_Cmd(USART3,ENABLE);
    //initialize the DMA channel.
    //initialize the DMA channel.
    //******************************************************************
    //USART    DMA接收
    DMA_Config( USART3_DMA_Steam_RX,
                USART3_DMA_Channel,
                (uint32_t)&(USART3->DR),     //串口DR寄存器
                (uint32_t)DMA_USART3_RX_Buf,//自定义的接收数据buf
                DMA_DIR_PeripheralToMemory,//外设到存储器方向
                DMA_USART3_RX_BUF_SIZE);//长度
    //******************************************************************
    //USART    DMA发送
    DMA_Config( USART3_DMA_Steam_TX,
                USART3_DMA_Channel,
                (uint32_t)&(USART3->DR),     //串口DR寄存器
                (uint32_t)DMA_USART3_TX_Buf,//自定义的接收数据buf
                DMA_DIR_MemoryToPeripheral,//存储器到外设方向
                0);//长度
    //******************************************************************
//    USART3_DISABLE();
    USART3_ENABLE();
}
void USART3_DISABLE(void)
{
    USART_ITConfig(USART3,USART_IT_IDLE,DISABLE);//关闭空闲中断
    USART_DMACmd(USART3,USART_DMAReq_Rx,DISABLE);//关闭DMA接收
    USART_DMACmd(USART3,USART_DMAReq_Tx,DISABLE);//DMA发送
    USART_Cmd(USART3,DISABLE);
}
void USART3_ENABLE(void)
{
    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);//空闲中断
    DMA_ITConfig(USART3_DMA_Steam_TX,DMA_IT_TC,ENABLE);//使能DMA传输完成中断
    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//DMA接收
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);//DMA发送
    USART_Cmd(USART3,ENABLE);
}
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    //#if CAN1_RX0_INT_ENABLE
    //   	NVIC_InitTypeDef  NVIC_InitStructure;
    //#endif
    //使能相关时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_CAN1 |RCC_APB1Periph_CAN2, ENABLE); //使能CAN1时钟
    //初始化GPIO
	
	//引脚复用映射配置
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); //
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); //
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //CAN单元设置
		CAN_DeInit(CAN1);
		CAN_StructInit(&CAN_InitStructure);
    CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
		CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  自动离线管理 
		CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
		CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
		CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
		CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
		CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
    CAN_InitStructure.CAN_SJW = tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1 = tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2 = tbs2; //Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler = brp; //分频系数(Fdiv)为brp+1
    CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1

    //配置过滤器
    CAN_FilterInitStructure.CAN_FilterNumber=0;						//筛选器组14
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
		/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
		//can总线标准ID是11位的,高对齐模式,000,0000,0000,xxxxx
		//											对应的数值 0000,0000,0000,0000
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0400;//;		//要筛选的ID高位 0x02x0 电机故障ID
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;// //要筛选的ID低位 
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0f00;// 0xFFFF;			//筛选器高16位每位必须匹配
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x000;// 0xFFFF;			//筛选器低16位每位必须匹配
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
		CAN_FilterInit(&CAN_FilterInitStructure);
		
		CAN_FilterInitStructure.CAN_FilterNumber=1;						//筛选器组14
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
		/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0A00;//;		//要筛选的ID高位 0xx5x0 电机参数读取地址
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;// //要筛选的ID低位 
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0f00;// 0xFFFF;			//筛选器高16位每位必须匹配
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x000;// 0xFFFF;			//筛选器低16位每位必须匹配
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
		CAN_FilterInit(&CAN_FilterInitStructure);	
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); //FIFO0消息挂号中断允许
    return 0;
}
void CAN1_ID_Init(u16 Location_ID)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;     //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //位宽
    CAN_FilterInitStructure.CAN_FilterIdHigh = Location_ID<<5 ;////ID
    CAN_FilterInitStructure.CAN_FilterIdLow = 0 | CAN_ID_STD;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = ((Local_CAN_ExtId<<3)>>16) & 0xffff;//MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = ((Local_CAN_ExtId<<3)& 0xffff) | CAN_ID_EXT;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
}
void NVIC_Configuration(void)
{
    ////////////////////////////////////////////////////////////////////////
    /*当两个中断源的抢占式优先级相同时，这两个中断将没有嵌套关系，当一个中断到来后，
      如果正在处理另一个中断，这个后到来的中断就要等到前一个中断处理完之后才能被处理。
      如果这两个中断同时到达，则中断控制器根据他们的响应优先级高低来决定先处理哪一个；
      如果他们的抢占式优先级和响应优先级都相等，则根据他们在中断表中的排位顺序决定先处理哪一个。
    */
    NVIC_InitTypeDef NVIC_InitStructure;
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH ,0x0);
//  /* Configure the NVIC Preemption Priority Bits */
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  //抢占优先级0位，从优先级4位
    //（先占优先级：高级中断可以打断低级中断，从优先级：同时中断响应高优先级，高优先级中断不能打断低优先级中断）
    /* Enable the USARTy Interrupt */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;    //能产生usart1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0; //从优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //使能中断
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 主优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;    //能产生usart2中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //使能中断
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;    //能产生usart3中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  //使能中断
    NVIC_Init(&NVIC_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;    //能产生TIME6中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    //使能中断
//    NVIC_Init(&NVIC_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;    //能产生TIME7中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    //使能中断
//    NVIC_Init(&NVIC_InitStructure);
//    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;   //能产生TIME10中断
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    //使能中断
//    NVIC_Init(&NVIC_InitStructure);
}
//***********************************************************************************
//
//                                  Encoder Functions
//
//***********************************************************************************
u16 Encoder_SSI_Pan(void)
{
    u32 d = 0;
    u8 i = 0,flag = 0;
    Delay_xUS(Encoder_Delay_US);
    Pan_SSI_CS_L;
    Delay_xUS(Encoder_Delay_US);
    Pan_SSI_CK_L;
    Delay_xUS(Encoder_Delay_US);
    d=0;
    for(i = 0; i < 12; i ++)
    {
        d = d << 1;
        Pan_SSI_CK_H;
        Delay_xUS(Encoder_Delay_US);
        if(Pan_SSI_DI)
        {
            d = d | 0x0001;
        }
        Pan_SSI_CK_L;
        Delay_xUS(Encoder_Delay_US);
    }
    flag = 0;
    for(i = 0; i < 6; i ++)
    {
        flag = flag << 1;
        Pan_SSI_CK_H;
        Delay_xUS(Encoder_Delay_US);
        if(Pan_SSI_DI)
        {
            flag = flag | 0x01;
        }
        Pan_SSI_CK_L;
        Delay_xUS(Encoder_Delay_US);
    }
    Pan_SSI_CS_H;
    Pan_SSI_CK_H;
    Pan_SSI_DO_H;
    if((flag & 0x38) == 0x20)
    {
        d = d;
        Pan_Encoder_State = Encoder_Normal;
    }
    else
    {
        d = 0xeeee;
        Pan_Encoder_State = Encoder_Error;
    }
    return d;//传递读取到的磁编码数值
}
u16 Encoder_SSI_Pan_M(void)//检测磁场强度值
{
    u32 d = 0;
    u8 i = 0,flag = 0;
    Delay_xUS(Encoder_Delay_US);
    Pan_SSI_CK_L;
    Delay_xUS(Encoder_Delay_US);
    Pan_SSI_CS_L;
    Delay_xUS(Encoder_Delay_US);
    d=0;
    for(i = 0; i < 12; i ++)
    {
        d = d << 1;
        Pan_SSI_CK_H;
        Delay_xUS(Encoder_Delay_US);
        if(Pan_SSI_DI)
        {
            d = d | 0x0001;
        }
        Pan_SSI_CK_L;
        Delay_xUS(Encoder_Delay_US);
    }
    flag = 0;
    for(i = 0; i < 6; i ++)
    {
        flag = flag << 1;
        Pan_SSI_CK_H;
        Delay_xUS(Encoder_Delay_US);
        if(Pan_SSI_DI)
        {
            flag = flag | 0x01;
        }
        Pan_SSI_CK_L;
        Delay_xUS(Encoder_Delay_US);
    }
    Pan_SSI_CS_H;
    Pan_SSI_CK_H;
    Pan_SSI_DO_H;
    if((flag & 0x38) == 0x20)
    {
        d = d;
        Pan_Encoder_State = Encoder_Normal;
    }
    else
    {
        d = 0xeeee;
        Pan_Encoder_State = Encoder_Error;
    }
    return d;//传递读取到的磁编码数值
}
u16 Encoder_SSI_Tilt(void)
{
    u32 d = 0;
    u8 i = 0,flag = 0;
    Delay_xUS(Encoder_Delay_US);
    Tilt_SSI_CS_L;
    Delay_xUS(Encoder_Delay_US);
    Tilt_SSI_CK_L;
    Delay_xUS(Encoder_Delay_US);
    d=0;
    for(i = 0; i < 12; i ++)
    {
        d = d << 1;
        Tilt_SSI_CK_H;
        Delay_xUS(Encoder_Delay_US);
        if(Tilt_SSI_DI)
        {
            d = d | 0x0001;
        }
        Tilt_SSI_CK_L;
        Delay_xUS(Encoder_Delay_US);
    }
    flag = 0;
    for(i = 0; i < 6; i ++)
    {
        flag = flag << 1;
        Tilt_SSI_CK_H;
        Delay_xUS(Encoder_Delay_US);
        if(Tilt_SSI_DI)
        {
            flag = flag | 0x01;
        }
        Tilt_SSI_CK_L;
        Delay_xUS(Encoder_Delay_US);
    }
    Tilt_SSI_CS_H;
    Tilt_SSI_CK_H;
    Tilt_SSI_DO_H;
    if((flag & 0x38) == 0x20)
    {
        d = d;
        Tilt_Encoder_State = Encoder_Normal;
    }
    else
    {
        d = 0xeeee;
        Tilt_Encoder_State = Encoder_Error;
    }
    return d;//传递读取到的磁编码数值
}
//
u32 pdata_test = 0;
u32 Encoder_SPI_Pan(u32 byte)
{
    u8 i;
    u32 p_data = 0;
    Pan_SSI_CS_L;
    Pan_SSI_CK_L;
    delay_us(1);
    for(i = 0; i < 32; i ++)
    {
        if(byte & 0x80000000)
        {
            Pan_SSI_DO_H;
        }
        else
        {
            Pan_SSI_DO_L;
        }
        delay_us(1);
        byte <<= 1;
        Pan_SSI_CK_H;
        delay_us(2);
        p_data <<= 1;
        if(Pan_SSI_DI)
        {
            p_data ++;
        }
        Pan_SSI_CK_L;
        delay_us(1);
    }
    p_data = p_data & 0x00ffffff;
    p_data = p_data >> 6;
    Pan_SSI_CS_H;
    return p_data;
}
u32 Encoder_SPI_Tilt(u32 byte)
{
    u8 i;
    u32 t_data = 0;
    //u32 f_data = 0;
    //f_data = byte;
    Tilt_SSI_CS_L;
    Tilt_SSI_CK_L;
    delay_us(1);
    for(i = 0; i < 32; i ++)
    {
        if(byte & 0x80000000)
        {
            Tilt_SSI_DO_H;
        }
        else
        {
            Tilt_SSI_DO_L;
        }
        delay_us(1);
        byte <<= 1;
        Tilt_SSI_CK_H;
        delay_us(2);
        t_data <<= 1;
        if(Tilt_SSI_DI)
        {
            t_data ++;
        }
        Tilt_SSI_CK_L;
        delay_us(1);
    }
    t_data = t_data & 0x00ffffff;
    t_data = t_data >> 6;
    Tilt_SSI_CS_H;
    return t_data;
}
//*****************************************************************
u16 Get_ID(void)
{
    u16 d = 0;
    u8 i;
    ID_CLK = 1;
    ID_LOAD = 1;
    Delay_xMS(1);
    ID_LOAD = 0;
    Delay_xMS(1);
    ID_LOAD = 1;
    Delay_xMS(1);
    for(i = 0; i < 12; i ++)
    {
        //d <<= 1;
        d >>= 1;
        ID_CLK = 0;
        Delay_xMS(1);
        if(!ID_QH)
        {
            d |= 0x800;
            //d |= 0x01;
        }
        ID_CLK = 1;
        Delay_xMS(1);
    }
    return d;
}
