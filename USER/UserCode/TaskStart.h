#ifndef _TaskStart_H
#define _TaskStart_H
#include "TaskMap.h"
extern OS_TCB  Task_Start_TCB;                       //ÈÎÎñ¿ØÖÆ¿é
void Task_Start(void *p);
uint32_t USART_BAND_Extract(uint32_t USART_BAUDRATE);
#endif
