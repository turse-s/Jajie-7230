#ifndef _hardware_H
#define _hardware_H
#include "includes.h"
#include "parameters.h"
#include "sys.h"
//***********************************************************************************
//
//                                  Hardware Interface
//
//***********************************************************************************
typedef struct GPIO_port
{
    GPIO_TypeDef*   GPIOx;
    uint32_t        RCC_AHB1Periph_GPIOx;
    uint16_t        GPIO_Pin_x;
} GPIO_port_t;
typedef struct SPIx_port
{
    GPIO_TypeDef*   GPIOx;
    uint32_t        RCC_AHB1Periph_GPIOx;
    uint16_t        SPI_NSS_Pin;
    uint16_t        SPI_SCK_Pin;
    uint16_t        SPI_MISO_Pin;
    uint16_t        SPI_MOSI_Pin;
} SPIx_port_t;
//***********************************************************************************
//                                  extern
//***********************************************************************************
extern GPIO_port_t LED1_port;
extern GPIO_port_t LED2_port;
extern SPIx_port_t Pan_SPI_port;
extern SPIx_port_t Tilt_SPI_port;
extern GPIO_port_t Optocoupler_port;
extern GPIO_port_t Relay_port;
#ifndef FOC
//***********************************************************************************
//                                  Power
//***********************************************************************************
//电机驱动电源
#define Drive_Switch_GPIO   GPIOA                   //
#define Drive_Switch_RCC    RCC_AHB1Periph_GPIOA    //
#define Drive_Switch_Pin    GPIO_Pin_15
#define Drive_Switch_ON     GPIO_SetBits(Drive_Switch_GPIO,Drive_Switch_Pin)
#define Drive_Switch_OFF    GPIO_ResetBits(Drive_Switch_GPIO,Drive_Switch_Pin)
//上行电源
#define UP_Switch_GPIO      GPIOC                   //
#define UP_Switch_RCC       RCC_AHB1Periph_GPIOC    //
#define UP_Switch_Pin       GPIO_Pin_10
#define UP_Switch_ON        GPIO_SetBits(UP_Switch_GPIO,UP_Switch_Pin)
#define UP_Switch_OFF       GPIO_ResetBits(UP_Switch_GPIO,UP_Switch_Pin)
//风扇电源
#define Heat_Switch_GPIO    GPIOC                   //
#define Heat_Switch_RCC     RCC_AHB1Periph_GPIOC    //
#define Heat_Switch_Pin     GPIO_Pin_12
#define Heat_Switch_ON      GPIO_SetBits(Heat_Switch_GPIO,Heat_Switch_Pin)
#define Heat_Switch_OFF     GPIO_ResetBits(Heat_Switch_GPIO,Heat_Switch_Pin)
//加热电源
#define Fan_Switch_GPIO     GPIOC                   //
#define Fan_Switch_RCC      RCC_AHB1Periph_GPIOC    //
#define Fan_Switch_Pin      GPIO_Pin_13
#define Fan_Switch_ON       GPIO_SetBits(Fan_Switch_GPIO,Fan_Switch_Pin)
#define Fan_Switch_OFF      GPIO_ResetBits(Fan_Switch_GPIO,Fan_Switch_Pin)
#endif
//***********************************************************************************
//                                  LEDS
//***********************************************************************************
#ifdef FOC
#define LED_GPIO    GPIOE
#define LED_RCC     RCC_AHB1Periph_GPIOE
#define LED2_Pin    GPIO_Pin_2 //
//#define LED3_Pin    GPIO_Pin_3 //
#define LED2    PEout(2)
//#define LED3    PCout(3)
#else
#define LED_GPIO    GPIOC
#define LED_RCC     RCC_AHB1Periph_GPIOC
#define LED2_Pin    GPIO_Pin_2 //
#define LED3_Pin    GPIO_Pin_3 //
#define LED2    PCout(2)
#define LED3    PCout(3)
//#define Relay1  PBout(6)
#endif
//***********************************************************************************
//                                  Switch
//***********************************************************************************
#define ID_LOAD_Pin     GPIO_Pin_0
#define ID_LOAD_RCC     RCC_AHB1Periph_GPIOB
#define ID_LOAD_GPIO    GPIOB
#define ID_CLK_Pin      GPIO_Pin_5
#define ID_CLK_RCC      RCC_AHB1Periph_GPIOC
#define ID_CLK_GPIO     GPIOC
#define ID_QH_Pin       GPIO_Pin_4
#define ID_QH_RCC       RCC_AHB1Periph_GPIOC
#define ID_QH_GPIO      GPIOC
#define ID_LOAD     PBout(0)
#define ID_CLK      PCout(5)
#define ID_QH       PCin(4)
#ifdef LASER_SETUP
//***********************************************************************************
//                                  Laser
//***********************************************************************************
#define LDR_Pin         GPIO_Pin_4
#define LDR_RCC         RCC_AHB1Periph_GPIOA
#define LDR_GPIO        GPIOA
#define LDR_Status      PAin(4)
#endif
//***********************************************************************************
//                                  USART
//***********************************************************************************
#define USART_DMA_Channel   DMA_Channel_4
//USART1
#define USART1_GPIO         GPIOA
#define USART1_RCC          RCC_AHB1Periph_GPIOA
#define USART1_Pin_TX       GPIO_Pin_9
#define USART1_Pin_RX       GPIO_Pin_10
#ifdef FOC
#define USART1_Pin_EN       GPIO_Pin_11
#define USART1_EN						PAout(11)
#endif
#define USART1_PinSource_TX GPIO_PinSource9  //PA9
#define USART1_PinSource_RX GPIO_PinSource10  //PA10
#define USART1_DMA_Steam_RX DMA2_Stream2
#define USART1_DMA_Steam_TX DMA2_Stream7
#define USART1_DMA_Channel  USART_DMA_Channel
//USART2
#define USART2_GPIO         GPIOD
#define USART2_RCC          RCC_AHB1Periph_GPIOD
#define USART2_TX_PinAF     GPIOD,GPIO_PinSource5,GPIO_AF_USART2
#define USART2_RX_PinAF     GPIOD,GPIO_PinSource6,GPIO_AF_USART2
#define USART2_Pin_TX       GPIO_Pin_5
#define USART2_Pin_RX       GPIO_Pin_6
#define USART2_DMA_Steam_RX DMA1_Stream5
#define USART2_DMA_Steam_TX DMA1_Stream6
#define USART2_DMA_Channel  USART_DMA_Channel
//USART3
#ifdef FOC
#define USART3_GPIO         GPIOB
#define USART3_RCC          RCC_AHB1Periph_GPIOB
#define USART3_TX_PinAF     GPIOB,GPIO_PinSource10,GPIO_AF_USART3
#define USART3_RX_PinAF     GPIOB,GPIO_PinSource11,GPIO_AF_USART3
#define USART3_Pin_TX       GPIO_Pin_10
#define USART3_Pin_RX       GPIO_Pin_11
#define USART3_DMA_Steam_RX DMA1_Stream1
#define USART3_DMA_Steam_TX DMA1_Stream3
#define USART3_DMA_Channel  USART_DMA_Channel
#else
#define USART3_GPIO         GPIOD
#define USART3_RCC          RCC_AHB1Periph_GPIOD
#define USART3_TX_PinAF     GPIOD,GPIO_PinSource8,GPIO_AF_USART3
#define USART3_RX_PinAF     GPIOD,GPIO_PinSource9,GPIO_AF_USART3
#define USART3_Pin_TX       GPIO_Pin_8
#define USART3_Pin_RX       GPIO_Pin_9
#define USART3_DMA_Steam_RX DMA1_Stream1
#define USART3_DMA_Steam_TX DMA1_Stream3
#define USART3_DMA_Channel  USART_DMA_Channel
#endif
//***********************************************************************************
//                                  Encoders
//***********************************************************************************
#ifdef FOC
//Encoder_Tilt
#define Tilt_SSI_CS_GPIO GPIOA                   //
#define Tilt_SSI_CS_RCC  RCC_AHB1Periph_GPIOA    //
#define Tilt_SSI_CS_Pin  GPIO_Pin_15              //
#define Tilt_SSI_CK_GPIO GPIOC                   //
#define Tilt_SSI_CK_RCC  RCC_AHB1Periph_GPIOC    //
#define Tilt_SSI_CK_Pin  GPIO_Pin_10              //
#define Tilt_SSI_DI_GPIO GPIOC                   //
#define Tilt_SSI_DI_RCC  RCC_AHB1Periph_GPIOC   //
#define Tilt_SSI_DI_Pin  GPIO_Pin_11             //
#define Tilt_SSI_DO_GPIO GPIOC                   //
#define Tilt_SSI_DO_RCC  RCC_AHB1Periph_GPIOC   //
#define Tilt_SSI_DO_Pin  GPIO_Pin_12
#define Tilt_Index_GPIO  GPIOA                   //
#define Tilt_Index_RCC   RCC_AHB1Periph_GPIOA    //
#define Tilt_Index_Pin   GPIO_Pin_15             //
#define Tilt_Dir_GPIO    GPIOD                   //
#define Tilt_Dir_RCC     RCC_AHB1Periph_GPIOD    //
#define Tilt_Dir_Pin     GPIO_Pin_3              //
#define Tilt_SSI_CS_H    GPIO_SetBits(Tilt_SSI_CS_GPIO,Tilt_SSI_CS_Pin)
#define Tilt_SSI_CS_L    GPIO_ResetBits(Tilt_SSI_CS_GPIO,Tilt_SSI_CS_Pin)
#define Tilt_SSI_CK_H    GPIO_SetBits(Tilt_SSI_CK_GPIO,Tilt_SSI_CK_Pin)
#define Tilt_SSI_CK_L    GPIO_ResetBits(Tilt_SSI_CK_GPIO,Tilt_SSI_CK_Pin)
#define Tilt_SSI_DI_H    GPIO_SetBits(Tilt_SSI_DI_GPIO,Tilt_SSI_DI_Pin)
#define Tilt_SSI_DI_L    GPIO_ResetBits(Tilt_SSI_DI_GPIO,Tilt_SSI_DI_Pin)
#define Tilt_SSI_DO_H    GPIO_SetBits(Tilt_SSI_DO_GPIO,Tilt_SSI_DO_Pin)
#define Tilt_SSI_DO_L    GPIO_ResetBits(Tilt_SSI_DO_GPIO,Tilt_SSI_DO_Pin)
#define Tilt_SSI_DI      GPIO_ReadInputDataBit(Tilt_SSI_DI_GPIO,Tilt_SSI_DI_Pin)
//#define Tilt_Dir         PDin(3)
//Encoder_Pan
#define Pan_SSI_CS_GPIO    GPIOB                   //
#define Pan_SSI_CS_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_CS_Pin     GPIO_Pin_12              //
#define Pan_SSI_CK_GPIO    GPIOB                   //
#define Pan_SSI_CK_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_CK_Pin     GPIO_Pin_13              //
#define Pan_SSI_DI_GPIO    GPIOB                   //
#define Pan_SSI_DI_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_DI_Pin     GPIO_Pin_14              //
#define Pan_SSI_DO_GPIO    GPIOB                   //
#define Pan_SSI_DO_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_DO_Pin     GPIO_Pin_15
#define Pan_Index_GPIO     GPIOD                   //
#define Pan_Index_RCC      RCC_AHB1Periph_GPIOD    //
#define Pan_Index_Pin      GPIO_Pin_2             //
#define Pan_Dir_GPIO       GPIOD                   //
#define Pan_Dir_RCC        RCC_AHB1Periph_GPIOD    //
#define Pan_Dir_Pin        GPIO_Pin_4              //
#define Pan_SSI_CS_H   GPIO_SetBits(Pan_SSI_CS_GPIO,Pan_SSI_CS_Pin)
#define Pan_SSI_CS_L   GPIO_ResetBits(Pan_SSI_CS_GPIO,Pan_SSI_CS_Pin)
#define Pan_SSI_CK_H   GPIO_SetBits(Pan_SSI_CK_GPIO,Pan_SSI_CK_Pin)
#define Pan_SSI_CK_L   GPIO_ResetBits(Pan_SSI_CK_GPIO,Pan_SSI_CK_Pin)
#define Pan_SSI_DI_H   GPIO_SetBits(Pan_SSI_DI_GPIO,Pan_SSI_DI_Pin)
#define Pan_SSI_DI_L   GPIO_ResetBits(Pan_SSI_DI_GPIO,Pan_SSI_DI_Pin)
#define Pan_SSI_DO_H   GPIO_SetBits(Pan_SSI_DO_GPIO,Pan_SSI_DO_Pin)
#define Pan_SSI_DO_L   GPIO_ResetBits(Pan_SSI_DO_GPIO,Pan_SSI_DO_Pin)
#define Pan_SSI_DI     GPIO_ReadInputDataBit(Pan_SSI_DI_GPIO,Pan_SSI_DI_Pin)
//#define Pan_Dir         PDin(4)
#else
//Encoder_Tilt
#define Tilt_SSI_CS_GPIO GPIOA                   //
#define Tilt_SSI_CS_RCC  RCC_AHB1Periph_GPIOA    //
#define Tilt_SSI_CS_Pin  GPIO_Pin_4              //
#define Tilt_SSI_CK_GPIO GPIOA                   //
#define Tilt_SSI_CK_RCC  RCC_AHB1Periph_GPIOA    //
#define Tilt_SSI_CK_Pin  GPIO_Pin_5              //
#define Tilt_SSI_DI_GPIO GPIOA                   //
#define Tilt_SSI_DI_RCC  RCC_AHB1Periph_GPIOA    //
#define Tilt_SSI_DI_Pin  GPIO_Pin_6              //
#define Tilt_SSI_DO_GPIO GPIOA                   //
#define Tilt_SSI_DO_RCC  RCC_AHB1Periph_GPIOA    //
#define Tilt_SSI_DO_Pin  GPIO_Pin_7
#define Tilt_Index_GPIO  GPIOA                   //
#define Tilt_Index_RCC   RCC_AHB1Periph_GPIOA    //
#define Tilt_Index_Pin   GPIO_Pin_15             //
#define Tilt_Dir_GPIO    GPIOD                   //
#define Tilt_Dir_RCC     RCC_AHB1Periph_GPIOD    //
#define Tilt_Dir_Pin     GPIO_Pin_3              //
#define Tilt_SSI_CS_H    GPIO_SetBits(Tilt_SSI_CS_GPIO,Tilt_SSI_CS_Pin)
#define Tilt_SSI_CS_L    GPIO_ResetBits(Tilt_SSI_CS_GPIO,Tilt_SSI_CS_Pin)
#define Tilt_SSI_CK_H    GPIO_SetBits(Tilt_SSI_CK_GPIO,Tilt_SSI_CK_Pin)
#define Tilt_SSI_CK_L    GPIO_ResetBits(Tilt_SSI_CK_GPIO,Tilt_SSI_CK_Pin)
#define Tilt_SSI_DI_H    GPIO_SetBits(Tilt_SSI_DI_GPIO,Tilt_SSI_DI_Pin)
#define Tilt_SSI_DI_L    GPIO_ResetBits(Tilt_SSI_DI_GPIO,Tilt_SSI_DI_Pin)
#define Tilt_SSI_DO_H    GPIO_SetBits(Tilt_SSI_DO_GPIO,Tilt_SSI_DO_Pin)
#define Tilt_SSI_DO_L    GPIO_ResetBits(Tilt_SSI_DO_GPIO,Tilt_SSI_DO_Pin)
#define Tilt_SSI_DI      GPIO_ReadInputDataBit(Tilt_SSI_DI_GPIO,Tilt_SSI_DI_Pin)
#define Tilt_Dir         PDin(3)
//Encoder_Pan
#define Pan_SSI_CS_GPIO    GPIOB                   //
#define Pan_SSI_CS_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_CS_Pin     GPIO_Pin_12              //
#define Pan_SSI_CK_GPIO    GPIOB                   //
#define Pan_SSI_CK_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_CK_Pin     GPIO_Pin_13              //
#define Pan_SSI_DI_GPIO    GPIOB                   //
#define Pan_SSI_DI_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_DI_Pin     GPIO_Pin_14              //
#define Pan_SSI_DO_GPIO    GPIOB                   //
#define Pan_SSI_DO_RCC     RCC_AHB1Periph_GPIOB    //
#define Pan_SSI_DO_Pin     GPIO_Pin_15
#define Pan_Index_GPIO     GPIOD                   //
#define Pan_Index_RCC      RCC_AHB1Periph_GPIOD    //
#define Pan_Index_Pin      GPIO_Pin_2             //
#define Pan_Dir_GPIO       GPIOD                   //
#define Pan_Dir_RCC        RCC_AHB1Periph_GPIOD    //
#define Pan_Dir_Pin        GPIO_Pin_4              //
#define Pan_SSI_CS_H   GPIO_SetBits(Pan_SSI_CS_GPIO,Pan_SSI_CS_Pin)
#define Pan_SSI_CS_L   GPIO_ResetBits(Pan_SSI_CS_GPIO,Pan_SSI_CS_Pin)
#define Pan_SSI_CK_H   GPIO_SetBits(Pan_SSI_CK_GPIO,Pan_SSI_CK_Pin)
#define Pan_SSI_CK_L   GPIO_ResetBits(Pan_SSI_CK_GPIO,Pan_SSI_CK_Pin)
#define Pan_SSI_DI_H   GPIO_SetBits(Pan_SSI_DI_GPIO,Pan_SSI_DI_Pin)
#define Pan_SSI_DI_L   GPIO_ResetBits(Pan_SSI_DI_GPIO,Pan_SSI_DI_Pin)
#define Pan_SSI_DO_H   GPIO_SetBits(Pan_SSI_DO_GPIO,Pan_SSI_DO_Pin)
#define Pan_SSI_DO_L   GPIO_ResetBits(Pan_SSI_DO_GPIO,Pan_SSI_DO_Pin)
#define Pan_SSI_DI     GPIO_ReadInputDataBit(Pan_SSI_DI_GPIO,Pan_SSI_DI_Pin)
#define Pan_Dir         PDin(4)
#endif
#ifndef FOC
//Motor_Pan
#define Motor_Pan_DIR_GPIO  GPIOE                   //
#define Motor_Pan_DIR_RCC   RCC_AHB1Periph_GPIOE    //
#define Motor_Pan_DIR_Pin   GPIO_Pin_8              //
#define Motor_Pan_BRK_GPIO  GPIOE                   //
#define Motor_Pan_BRK_RCC   RCC_AHB1Periph_GPIOE    //
#define Motor_Pan_BRK_Pin   GPIO_Pin_7              //
#define Motor_Pan_PWM_GPIO  GPIOE                   //
#define Motor_Pan_PWM_RCC   RCC_AHB1Periph_GPIOE    //
#define Motor_Pan_PWM_Pin   GPIO_Pin_5              //
#define Motor_Pan_PWM_PinAF GPIOE,GPIO_PinSource5,GPIO_AF_TIM9
#define Motor_Pan_FG_GPIO   GPIOE                   //
#define Motor_Pan_FG_RCC    RCC_AHB1Periph_GPIOE    //
#define Motor_Pan_FG_Pin    GPIO_Pin_9              //
#define Motor_Pan_DIR(P_dir)    PEout(8) = P_dir
#define Motor_Pan_BRK(P_brk)    PEout(7) = P_brk
//#define Motor_Pan_EN(P_en)      PCout(5) = P_en
//Motor_Tilt
#define Motor_Tilt_DIR_GPIO     GPIOE                   //
#define Motor_Tilt_DIR_RCC      RCC_AHB1Periph_GPIOE    //
#define Motor_Tilt_DIR_Pin      GPIO_Pin_11             //
#define Motor_Tilt_BRK_GPIO     GPIOE                   //
#define Motor_Tilt_BRK_RCC      RCC_AHB1Periph_GPIOE    //
#define Motor_Tilt_BRK_Pin      GPIO_Pin_10             //
#define Motor_Tilt_PWM_GPIO     GPIOE                   //
#define Motor_Tilt_PWM_RCC      RCC_AHB1Periph_GPIOE    //
#define Motor_Tilt_PWM_Pin      GPIO_Pin_6              //
#define Motor_Tilt_PWM_PinAF    GPIOE,GPIO_PinSource6,GPIO_AF_TIM9
#define Motor_Tilt_FG_GPIO      GPIOE                   //
#define Motor_Tilt_FG_RCC       RCC_AHB1Periph_GPIOE    //
#define Motor_Tilt_FG_Pin       GPIO_Pin_12             //
#define Motor_Tilt_DIR(T_dir)   PEout(11) = T_dir
#define Motor_Tilt_BRK(T_brk)   PEout(10) = T_brk
//#define Motor_Tilt_EN(T_en)     PBout(15) = T_en
#endif
//***********************************************************************************
//                                  Calibrations
//***********************************************************************************
//Pan_Calibration
#define Pan_Calibration1_GPIO   GPIOB
#define Pan_Calibration1_RCC    RCC_AHB1Periph_GPIOB
#define Pan_Calibration1_Pin    GPIO_Pin_5
#define Pan_Calibration2_GPIO   GPIOB
#define Pan_Calibration2_RCC    RCC_AHB1Periph_GPIOB
#define Pan_Calibration2_Pin    GPIO_Pin_6
#define Pan_Calibration3_GPIO   GPIOB
#define Pan_Calibration3_RCC    RCC_AHB1Periph_GPIOB
#define Pan_Calibration3_Pin    GPIO_Pin_7
#define Pan_Calibration1        PBin(5)
#define Pan_Calibration2        PBin(6)
#define Pan_Calibration3        PBin(7)
//Tilt_Calibration
#define Tilt_Calibration1_GPIO   GPIOB
#define Tilt_Calibration1_RCC    RCC_AHB1Periph_GPIOB
#define Tilt_Calibration1_Pin    GPIO_Pin_9
#define Tilt_Calibration2_GPIO   GPIOE
#define Tilt_Calibration2_RCC    RCC_AHB1Periph_GPIOE
#define Tilt_Calibration2_Pin    GPIO_Pin_0
#define Tilt_Calibration3_GPIO   GPIOE
#define Tilt_Calibration3_RCC    RCC_AHB1Periph_GPIOE
#define Tilt_Calibration3_Pin    GPIO_Pin_1
//#define Tilt_Calibration_Middle PBin(9)
//#define Tilt_Calibration_Top    PEin(0)
//#define Tilt_Calibration_Bottom PEin(1)
#define Tilt_Calibration_Middle PEin(1)
#define Tilt_Calibration_Top    PEin(0)
#define Tilt_Calibration_Bottom PBin(9)
//***********************************************************************************
//                                  Function Declaration
//***********************************************************************************
void PWR_PVD_Init(void);
void IWDG_Init(u8 psc,u16 rld);
//Power
void Drive_Switch_Init(void);
void UP_Switch_Init(void);
void Heat_Switch_Init(void);
void Fan_Switch_Init(void);
//Leds
void GPIO_PORT_INIT(GPIO_port_t pHandle);
void GPIO_PORT_INIT_IN(GPIO_port_t pHandle);
void GET_ID_GPIO_INIT(void);
#ifdef LASER_SETUP
void Laser_Init(void);
#endif
#ifndef FOC
void Motor_Pan_Init(void);
void Motor_Tilt_Init(void);
void Motor_Pan_PWM_Init(u32 arr,u32 psc);
void Motor_Tilt_PWM_Init(u32 arr,u32 psc);
#endif
void SPI_PORT_INIT(SPIx_port_t pHandle);
#ifdef FOC
void SPI_PORT_INIT_Tilt(SPIx_port_t pHandle);
void SPIx_Init(SPI_TypeDef *SPIx,GPIO_TypeDef *SPI_PORT,uint16_t Pin_CLK,uint16_t Pin_MISO,uint16_t Pin_MOSI,GPIO_TypeDef *CS_PORT,uint16_t Pin_CS,uint8_t AF_pinCLK,uint8_t AF_pinMISO,uint8_t AF_pinMOSI);
#endif
void Encoder_Pan_Init(void);
void Encoder_Tilt_Init(void);
void Encoder_Pan_EXTI_Line2(FunctionalState state);
void Encoder_Tilt_EXTI_Line15(FunctionalState state);
void Pan_Calibration_Init(void);
void Pan_Calibration_EXTI_DISABLE(void);
void Tilt_Calibration_Init(void);
void Tilt_Calibration_EXTI_DISABLE(void);
void Tilt_Calibration_EXTI_ENABLE(void);
//MotorPortInitialize
void TIM2_Init(u16 arr,u16 psc);
void TIM3_Init(u16 arr,u16 psc);
#ifndef FOC
void TIM4_Init(u16 arr,u16 psc);//正交编码器模式，垂直
void TIM5_Init(u16 arr,u16 psc);//正交编码器模式，水平
#endif
void TIM6_Init(u16 arr,u16 psc);
void TIM7_Init(u16 arr,u16 psc);
void TIM10_Init(u16 arr,u16 psc);
void Motor_Pan_Init(void);
void Motor_Tilt_Init(void);
//USARTPortInitialize
void DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,uint32_t DMA_DIR_MP,u16 ndtr);
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);//开启一次DMA传输
void USART1_INT_Init(u32 bound);
void USART1_DISABLE(void);
void USART1_ENABLE(void);
void USART2_INT_Init(u32 bound);
void USART2_DISABLE(void);
void USART2_ENABLE(void);
void USART3_INT_Init(u32 bound);
void USART3_DISABLE(void);
void USART3_ENABLE(void);
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void CAN1_ID_Init(u16 Location_ID);
void NVIC_Configuration(void);
//Encoders
u16 Encoder_SSI_Pan(void);
u16 Encoder_SSI_Pan_M(void);//检测磁场强度值
u16 Encoder_SSI_Tilt(void);
u32 Encoder_SPI_Pan(u32 byte);
u32 Encoder_SPI_Tilt(u32 byte);
#endif
