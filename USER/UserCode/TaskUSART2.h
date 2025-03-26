#ifndef _TaskUSART2_H
#define _TaskUSART2_H
#include "parameters.h"
void Task_USART2_RX(void *p);
void USART2_RX_Function(void);
void    USART2_RX_Analyze(void);
void USART2_Special_Preset_Set(u16 num);
void USART2_Special_Preset_Delete(u16 num);
void USART2_Special_Preset_Extract(u16 num);
u32 USART2_Parameters_Init_U2(void);
u16 USART2_Speed_Level_Pan(u16 speed,u8 state_Self_Adaption);
u16 USART2_Speed_Level_Tilt(u16 speed,u8 state_Self_Adaption);
#endif
