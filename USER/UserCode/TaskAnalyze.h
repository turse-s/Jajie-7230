#ifndef _TaskAnalyze_H
#define _TaskAnalyze_H
#include "parameters.h"
void Task_Analyze(void *p);
void Analyze_Function(void);
void CMD_Analyze(u16 Down_CMD,u32 Down_Data,int usart);
void Special_Preset_Set(u16 num);
void Special_Preset_Delete(u16 num);
void Special_Preset_Extract(u16 num);
u32 Parameters_Init(void);
extern int zoom_record;
#endif
