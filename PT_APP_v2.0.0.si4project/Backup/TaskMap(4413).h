#ifndef _TaskMap_H
#define _TaskMap_H
#include "includes.h"
//任务分布图
//Start
#define Task_Start_PRIO             15
#define Task_Start_Stack_Size       128
//Analyze
#define Task_Analyze_PRIO           4
#define Task_Analyze_Stack_Size     2048
//CAN
#define Task_CAN_PRIO               5
#define Task_CAN_Stack_Size         1024
//USART_TX
#define Task_USART_TX_PRIO          6
#define Task_USART_TX_Stack_Size    1024
//Encoder
#define Task_Encoder_PRIO           7
#define Task_Encoder_Stack_Size     512
//USART1_RX
#define Task_USART1_RX_PRIO         8
#define Task_USART1_RX_Stack_Size   2048
//USART2_RX
#define Task_USART2_RX_PRIO         10
#define Task_USART2_RX_Stack_Size   256
//USART3_RX
#define Task_USART3_RX_PRIO         9
#define Task_USART3_RX_Stack_Size   256
//Timer
#define Task_Timer_PRIO             11
#define Task_Timer_Stack_Size       512
//LedState
#define Task_LedState_PRIO          12
#define Task_LedState_Stack_Size    256
#endif
