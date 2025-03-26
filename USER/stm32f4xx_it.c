/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "parameters.h"
#include "stm32f4xx_it.h"
#include "TaskMotor.h"
#include "TaskUSART1.h"
#include "TaskUSART3.h"
#include "TaskEncoder.h"
#include "TaskCAN.h"
#include "hardware.h"
#include "stmflash.h"
#include "TaskEncoder.h"
#include "JQueue.h"
#define Upgrade_Data_length  36
u8 Remote_upgrade=0;
u8  Receive_Data[300];
u16 Accept_Long=0;	




//static u8 usart1_buffer;	
extern u8 Ack_State[1];  /*! 应答状态  */
u8 write_flash_add;
/******* 命令缓冲区  *************/	
u32 Upgrade_command_buffer;

u32 Read_command_adderss;	
u32 Go_command_adderss;
u32 Upgrade_address;
u32 Erase_address;

u16 Upgrade_buffer_length;
u32 Upgrade_total_length_byte=0;
u32 exit_wrie_adderss;
u8 Ptz_Leans_state;

u16 Updata_Command;
u8 Updata_Command_state;
u8 ACK_STATE;

/*读取数据长度*/
u8 Read_Data_Number;

/***  ***/
//void DMA1_Channel3_IRQHandler(void)  
//{  
//	
//} 


//void TIM2_IRQHandler(void)
//{	
//	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//	{  
//		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//		
//	}
//}
//u16 USART3_Accept_Long = 0;
//void TIM3_IRQHandler(void)
//{	
//	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
//	{  
//		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
//		
//    }
//}

//void TIM4_IRQHandler(void)
//{	
//	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
//	{  
//		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
//		
//	}
//}
//void TIM6_DAC_IRQHandler(void)
//{	
//	if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
//	{  
//		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
////		LED3 = ~LED3;
//        Motor_Speed_Pan();
////        EEPRom_ReadDataLen(0,1,cc,8);
//	}
//}
//void TIM7_IRQHandler(void)
//{	
//	if(TIM_GetITStatus(TIM7,TIM_IT_Update) != RESET)
//	{  
//		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
////		LED2 = ~LED2;
//        Motor_Speed_Tilt();
//	}
//}
u8 pwr_flag = 0;

//void PVD_IRQHandler(void) 
//{
//    OSIntEnter();
//    if(EXTI_GetITStatus(EXTI_Line16) != RESET)
//    {
//        EXTI_ClearFlag(EXTI_Line16);
//        EXTI_ClearITPendingBit(EXTI_Line16);
//        if(!pwr_flag)
//        {
//            pwr_flag = 1;
//            PWR_PVDCmd(DISABLE);
//			STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
////			Encoder_Position_Storage();
//        }
//    }
//    OSIntExit();
//}

///**********************************
//* 功能：串口中断服务程序
//* 描述：中断模式接收命令
//**********************************/
void USART1_IRQHandler(void) 
{
    OS_ERR err;
    volatile uint8_t rc_tmp;
    uint16_t rc_len;
    uint16_t i;
    OSIntEnter();
    if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
    {
        rc_tmp = USART1->SR;
        rc_tmp = USART1->DR;//软件序列清除IDLE标志位
        DMA_Cmd(USART1_DMA_Steam_RX,DISABLE);//关闭DMA，准备重新配置
        DMA_ClearITPendingBit(USART1_DMA_Steam_RX,DMA_IT_TCIF2);	// Clear Transfer Complete flag
        DMA_ClearITPendingBit(USART1_DMA_Steam_RX,DMA_IT_TEIF2);	// Clear Transfer error flag	
        rc_len = DMA_USART1_RX_BUF_SIZE - DMA_GetCurrDataCounter(USART1_DMA_Steam_RX);//计算接收数据长度
        for(i = 0;i < rc_len;i ++)
        {
            USART1_RX_Buf[USART1_data_pointer ++] = DMA_USART1_RX_Buf[i];
            if(USART1_data_pointer == DMA_USART1_RX_BUF_SIZE)
            {
                USART1_data_pointer = 0;
            }
        }
        LED2 = ON;
        OSSemPost(&Sem_USART1_RX,OS_OPT_POST_1,&err);
//        Post_Information_Function(Sem_USART1_RX,&USART1_RX_BOX,0,DMA_USART1_RX_Buf,rc_len);
    }
    DMA_SetCurrDataCounter(USART1_DMA_Steam_RX,DMA_USART1_RX_BUF_SIZE);          //设置传输数据长度 
	DMA_Cmd(USART1_DMA_Steam_RX,ENABLE);                      //开启DMA
    OSIntExit();
}
void USART2_IRQHandler(void) 
{
    OS_ERR err;
    volatile uint8_t rc_tmp;
    uint16_t rc_len;
    uint16_t i;
    OSIntEnter();
    if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
    {
        rc_tmp = USART2->SR;
        rc_tmp = USART2->DR;//软件序列清除IDLE标志位
        DMA_Cmd(USART2_DMA_Steam_RX,DISABLE);//关闭DMA，准备重新配置
        DMA_ClearITPendingBit(USART2_DMA_Steam_RX,DMA_IT_TCIF5);	// Clear Transfer Complete flag
        DMA_ClearITPendingBit(USART2_DMA_Steam_RX,DMA_IT_TEIF5);	// Clear Transfer error flag	
        rc_len = DMA_USART2_RX_BUF_SIZE - DMA_GetCurrDataCounter(USART2_DMA_Steam_RX);//计算接收数据长度
        for(i = 0;i < rc_len;i ++)//输出每一字节的数据观察
        {
            USART2_RX_Buf[USART2_data_pointer ++] = DMA_USART2_RX_Buf[i];
            if(USART2_data_pointer == DMA_USART2_RX_BUF_SIZE)                    //数据存到队尾
            {
                USART2_data_pointer = 0;                       //转到队首
            }
        }
        LED2 = ON;
        OSSemPost(&Sem_USART2_RX,OS_OPT_POST_1,&err);
	}
    DMA_SetCurrDataCounter(USART2_DMA_Steam_RX,DMA_USART2_RX_BUF_SIZE);          //设置传输数据长度 
	DMA_Cmd(USART2_DMA_Steam_RX,ENABLE);                      //开启DMA
    OSIntExit();
}
void USART3_IRQHandler(void)
{
    OS_ERR err;
    volatile uint8_t rc_tmp;
    uint16_t rc_len;
    uint16_t i;
    OSIntEnter();
    if(USART_GetITStatus(USART3,USART_IT_IDLE)!=RESET)
    {
        rc_tmp = USART3->SR;
        rc_tmp = USART3->DR;//软件序列清除IDLE标志位
        DMA_Cmd(USART3_DMA_Steam_RX, DISABLE);//关闭DMA，准备重新配置
        DMA_ClearITPendingBit(USART3_DMA_Steam_RX,DMA_IT_TCIF1);	// Clear Transfer Complete flag
        DMA_ClearITPendingBit(USART3_DMA_Steam_RX,DMA_IT_TEIF1);	// Clear Transfer error flag

        rc_len = DMA_USART3_RX_BUF_SIZE - DMA_GetCurrDataCounter(USART3_DMA_Steam_RX);//计算接收数据长度
//        memcpy(USART3_RX_Buf,Empty_Buf,10);
        for(i = 0;i < rc_len;i ++)//输出每一字节的数据观察
        {
            USART3_RX_Buf[USART3_data_pointer ++] = DMA_USART3_RX_Buf[i];
            if(USART3_data_pointer == DMA_USART3_RX_BUF_SIZE)                    //数据存到队尾
            {
                USART3_data_pointer = 0;                       //转到队首
            }
        }
        LED2 = ON;
        OSSemPost(&Sem_USART3_RX,OS_OPT_POST_1,&err);
	}
    DMA_SetCurrDataCounter(USART3_DMA_Steam_RX,DMA_USART3_RX_BUF_SIZE);          //设置传输数据长度 
	DMA_Cmd(USART3_DMA_Steam_RX,ENABLE);                      //开启DMA
    OSIntExit();
}

u8 CAN1_Data_Num = 0;
void CAN1_RX0_IRQHandler(void)
{
//    u32 i;
    CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);//读取数据
//    if(RxMessage.StdId == Local_CAN_StdId)
//    {
//        LED2 = ON;
//        for(i=0;i < RxMessage.DLC;i++)
//        {
//            Local_CAN_BUF[i] = RxMessage.Data[i];
//        }
//        CAN1_Data_Num = RxMessage.DLC;
//        CAN1_RX_Data(Local_CAN_BUF,CAN1_Data_Num);
//    }
    CAN_FIFORelease(CAN1,CAN_FIFO0);
}

void DMA2_Stream7_IRQHandler(void)
{
//    OS_ERR err;
    OSIntEnter();
    if(DMA_GetITStatus(USART1_DMA_Steam_TX,DMA_IT_TCIF7) != RESET)   
    {  
        DMA_ClearFlag(USART1_DMA_Steam_TX,DMA_IT_TCIF7);/* 清除标志位 */  
        DMA_Cmd(USART1_DMA_Steam_TX,DISABLE);/* 关闭DMA */
        USART1_Use_DMA_Tx_Flag = 0;/* 发送完成  */ 
//        OSSemPost(&Sem_USART1_TX,OS_OPT_POST_1,&err);
        Pend_Information_Function_TX(&USART1_TX_BOX);
    }
    OSIntExit();
}
void DMA1_Stream6_IRQHandler(void)
{
//    OS_ERR err;
    OSIntEnter();
    if(DMA_GetITStatus(USART2_DMA_Steam_TX,DMA_IT_TCIF6) != RESET)   
    {
        DMA_ClearFlag(USART2_DMA_Steam_TX,DMA_IT_TCIF6);/* 清除标志位 */
        DMA_Cmd(USART2_DMA_Steam_TX,DISABLE);/* 关闭DMA */
        USART2_Use_DMA_Tx_Flag = 0;/* 发送完成  */
//        OSSemPost(&Sem_USART2_TX,OS_OPT_POST_1,&err);
        Pend_Information_Function_TX(&USART2_TX_BOX);
    }
    OSIntExit();
}
void DMA1_Stream3_IRQHandler(void)
{
//    OS_ERR err;
    OSIntEnter();
    if(DMA_GetITStatus(USART3_DMA_Steam_TX,DMA_IT_TCIF3) != RESET)   
    {  
        DMA_ClearFlag(USART3_DMA_Steam_TX,DMA_IT_TCIF3);/* 清除标志位 */  
        DMA_Cmd(USART3_DMA_Steam_TX,DISABLE);/* 关闭DMA */
        USART3_Use_DMA_Tx_Flag = 0;/* 发送完成  */
//        OSSemPost(&Sem_USART3_TX,OS_OPT_POST_1,&err);
        Pend_Information_Function_TX(&USART3_TX_BOX);
    }
    OSIntExit();
}
void EXTI9_5_IRQHandler(void)//外部中断5~9线
{
    OSIntEnter();
    //*****************************************************
    //水平光管3
//    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
//    {
//        EXTI_ClearFlag(EXTI_Line5);
//        EXTI_ClearITPendingBit(EXTI_Line5);
//        if(Pan_Init_Flag == 1)
//        {
//            Pan_Init_Flag = 2;
//            Pan_Target_Speed = 0;
//            Pan_Section_Counter_Check = Pan_Section_Counter;
//            Pan_Section_Counter = 0;
//        }
//    }
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line7);
        EXTI_ClearITPendingBit(EXTI_Line7);
        
        if(Pan_Init_Flag == 1)
        {
            Pan_Init_Flag = 2;
            Pan_Target_Speed = 0;
            Pan_Section_Counter_Check = Pan_Section_Counter;
            Pan_Section_Counter = 0;
        }
//        if(Pan_Init_Flag == 3 && Pan_A_Speed > 0)
//        {
//            Pan_Section_Counter = 0;
//            Pan_Tour_Scan_Accumulation = 0;
//        }
    }
//    //*****************************************************
//    //垂直光管1
//    if(EXTI_GetITStatus(EXTI_Line9) != RESET)
//    {
//        EXTI_ClearFlag(EXTI_Line9);
//        EXTI_ClearITPendingBit(EXTI_Line9);
////        LED3 = !LED3;
//        if(Tilt_Init_Flag == 2)
//        {
//            
//            if(Tilt_Hard_Limit_FLAG == Tilt_Limit_None)
//            {
//                Tilt_Init_Flag = 3;
//                Tilt_Target_Speed = 0;
//                Tilt_Section_Counter_Check = Tilt_Section_Counter;
//                Tilt_Section_Counter = 0;
//                EXTI->IMR &= ~(EXTI_Line9);// 屏蔽外部中断9
//            }
//        }
//    }
   OSIntExit(); 
}
void EXTI1_IRQHandler(void)//外部中断1线
{
    OSIntEnter();
    //垂直光管1
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearFlag(EXTI_Line1);
        EXTI_ClearITPendingBit(EXTI_Line1);
//        LED3 = !LED3;
        if(Tilt_Init_Flag == 2)
        {
            if(Tilt_Hard_Limit_FLAG == Tilt_Limit_None)
            {
                Tilt_Init_Flag = 3;
                Tilt_Target_Speed = 0;
                Tilt_Section_Counter_Check = Tilt_Section_Counter;
                Tilt_Section_Counter = 0;
//                EXTI->IMR &= ~(EXTI_Line1);// 屏蔽外部中断1
            }
        }
		if(Origin_Machine_Flag == 3)
		{
			Origin_Machine_Flag = 4;
			Tilt_Target_Speed = 0;
			Origin_Machine_Record[4] = (u16)(Pan_Real_Angle_1 * 100) >> 8;
			Origin_Machine_Record[5] = (u16)(Pan_Real_Angle_1 * 100) & 0xff;
			Pan_Machine_Origin_Angle = Pan_Real_Angle_1;
			Origin_Machine_Record[6] = (u16)(Tilt_Real_Angle_1 * 100) >> 8;
			Origin_Machine_Record[7] = (u16)(Tilt_Real_Angle_1 * 100) & 0xff;
			Tilt_Machine_Origin_Angle = Tilt_Real_Angle_1;
		}
//        if(Tilt_Init_Flag == 4 && Tilt_A_Speed > 0)
//        {
//            Tilt_Section_Counter = 0;
//            Tilt_Tour_Scan_Accumulation = 0;
//        }
    }
    OSIntExit();
}

void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

 

 

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
