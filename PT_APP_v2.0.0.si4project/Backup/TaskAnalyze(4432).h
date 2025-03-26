#ifndef _TaskAnalyze_H
#define _TaskAnalyze_H
#include "parameters.h"
void Task_Analyze(void *p);
void Analyze_Function(void);
void Special_Preset_Set(u16 num);
void Special_Preset_Delete(u16 num);
void Special_Preset_Extract(u16 num);
u16 Speed_Level_Pan(u16 speed,u8 state_Self_Adaption);
u16 Speed_Level_Tilt(u16 speed,u8 state_Self_Adaption);
u32 Parameters_Init(void);
#endif
