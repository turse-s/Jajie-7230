#ifndef _TaskUSART1_H
#define _TaskUSART1_H
#include "parameters.h"
#define Alarm_Output  0x72
#define Alarm_Input   0x07
#define Alarm_Input_Setup 0x71
typedef struct
{
    int type;           //报警类型（常开报警or常闭报警）
    int status;         //报警状态（当前状态（检测端是开启还是关闭））
    int alert;          //报警开关（是否在报警）
    int enable;         //报警使能（是否开启报警功能）
} Alarm_Data;
extern Alarm_Data Alarm[];
void Task_USART1_RX(void *p);
void USART1_RX_Function(void);
void Task_USART1_TX(void *p);
void USART1_TX_Function(u32 cmd,u8 *data,u16 len);
void Alarm_Function_Analyze(void);
#endif
