#include "parameters.h"


extern int rocker_mark;
void Rocker_Control(u16 Down_CMD,u16 Down_Data);
int Rocker_Select(u16 Down_CMD,u16 Down_Data);
int CalculateSpeed(u16 speed, u8 state_Self_Adaption, int ultimate_min, int ultimate_max, int scale);
int Speed_Level_Pan(u16 speed,u8 state_Self_Adaption);
int Speed_Level_Tilt(u16 speed,u8 state_Self_Adaption);
