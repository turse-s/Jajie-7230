#include "RockerControl.h"
#include "sys.h"

///////////////////////////////////////////////////////
/*摇杆控制函数*/
void Rocker_Control(u16 Down_CMD,u16 Down_Data)
{
    switch(Down_CMD)
    {
    case Direction_Stop:  
        Pan_Motion_Mode = Rocker_Motion;                                    //设置水平摇杆模式
        Tilt_Motion_Mode = Rocker_Motion;                                   //设置垂直摇杆模式
        Pan_Target_Speed = 0;                                               //水平目标速度初始化
        Tilt_Target_Speed = 0;                                              //垂直目标速度初始化
        break;
    case Direction_Up:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef 

            Tilt_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1: Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
#ifdef REVERSE_TILT
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#endif
#endif
        break;
    case Direction_Down:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef ZS02
            Tilt_Target_Speed = (PTZ_Inverted)? Speed_Level_Tilt(Down_Data,Self_Adaption): ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
#ifdef REVERSE_TILT
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#endif
#endif
        break;
    case Direction_Left:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef ZS02
            Pan_Target_Speed = (PTZ_Inverted)? Speed_Level_Pan(Down_Data,Self_Adaption): ~Speed_Level_Pan(Down_Data,Self_Adaption) +1;
#else
#ifdef REVERSE_PAN
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#else
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#endif
#endif
        break;
    case Direction_Right:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef ZS02
            Pan_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Pan(Down_Data,Self_Adaption) +1: Speed_Level_Pan(Down_Data,Self_Adaption);
#else
#ifdef REVERSE_PAN
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#else
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#endif
#endif
        break;
    case Direction_Up_Left:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef ZS02
                        Tilt_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1: Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
#ifdef REVERSE_TILT
                        Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
                        Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#endif
#endif

#ifdef ZS02
                        Pan_Target_Speed = (PTZ_Inverted)? Speed_Level_Pan(Down_Data,Self_Adaption): ~Speed_Level_Pan(Down_Data,Self_Adaption) +1;
#else
#ifdef REVERSE_PAN
                        Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#else
                        Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#endif
#endif     
        break;
    case Direction_Up_Right:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef ZS02
                        Tilt_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1: Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
#ifdef REVERSE_TILT
                        Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
                        Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#endif
#endif

#ifdef ZS02
                        Pan_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Pan(Down_Data,Self_Adaption) +1: Speed_Level_Pan(Down_Data,Self_Adaption);
#else
#ifdef REVERSE_PAN
                        Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#else
                        Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#endif
#endif
        break;
    case Direction_Down_Left:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef ZS02
                        Tilt_Target_Speed = (PTZ_Inverted)? Speed_Level_Tilt(Down_Data,Self_Adaption): ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
#ifdef REVERSE_TILT
                        Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
                        Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#endif
#endif

#ifdef ZS02
                        Pan_Target_Speed = (PTZ_Inverted)? Speed_Level_Pan(Down_Data,Self_Adaption): ~Speed_Level_Pan(Down_Data,Self_Adaption) +1;
#else
#ifdef REVERSE_PAN
                        Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#else
                        Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#endif
#endif
        break;
    case Direction_Down_Right:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef ZS02
                        Tilt_Target_Speed = (PTZ_Inverted)? Speed_Level_Tilt(Down_Data,Self_Adaption): ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
#ifdef REVERSE_TILT
                        Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
                        Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#endif
#endif

#ifdef ZS02
                        Pan_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Pan(Down_Data,Self_Adaption) +1: Speed_Level_Pan(Down_Data,Self_Adaption);
#else
#ifdef REVERSE_PAN
                        Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#else
                        Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#endif
#endif
        break;
    default:break;
				}
}

u16 Speed_Level_Pan(u16 speed,u8 state_Self_Adaption)
{
    u16 sp = 0;
    u16 level = 0;
    u16 level_max = 0;
    level = speed >> 8;
    if(state_Self_Adaption)
    {
        if(Zoom_Position > Zoom_Position_Max)
        {
            Zoom_Position = Zoom_Position_Max;
        }
        if(Zoom_Position < Zoom_Position_Min)
        {
            Zoom_Position = Zoom_Position_Min;
        }
        level_max = (Zoom_Position_Max - Zoom_Position) * 64/ (Zoom_Position_Max - Zoom_Position_Min);
        if(level_max < 3)
        {
            level_max = 3;
        }
        if(level_max > 64)
        {
            level_max = 64;
        }
        if (level > level_max)
        {
            level = level_max;
        }
    }
    if((level) <= 64 && (level) != 0)
    {
        sp = (((Pan_Ultimate_MaxSpeed * Speed_Scale_Pan - (Pan_Ultimate_MinSpeed)) * (level)) + (Pan_Ultimate_MinSpeed * Speed_Scale_Pan) * 64 - Pan_Ultimate_MaxSpeed) / 63;
        if(sp < Pan_Ultimate_MinSpeed)
        {
            sp = Pan_Ultimate_MinSpeed;
        }
        if(sp > Pan_Ultimate_MaxSpeed * Speed_Scale_Pan)
        {
            sp = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan;
        }
    }
    Velocity_Pan = (level);
    Pan_Softstart = Velocity_Pan - 20;
    Pan_Softstart *= 5;
    return sp;
}
u16 Speed_Level_Tilt(u16 speed,u8 state_Self_Adaption)
{
    u16 sp = 0;
    u16 level_max;
    u16 level;
    level = speed & 0xff;
    if(state_Self_Adaption)
    {
        if(Zoom_Position > Zoom_Position_Max)
        {
            Zoom_Position = Zoom_Position_Max;
        }
        if(Zoom_Position < Zoom_Position_Min)
        {
            Zoom_Position = Zoom_Position_Min;
        }
        level_max = (Zoom_Position_Max - Zoom_Position) * 64 / (Zoom_Position_Max - Zoom_Position_Min);
        if(level_max < 3)
        {
            level_max = 3;
        }
        if(level_max > 64)
        {
            level_max = 64;
        }
        if (level > level_max)
        {
            level = level_max;
        }
    }
    if((level) <= 64 && (level) != 0)
    {
        sp = (((Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt - (Tilt_Ultimate_MinSpeed)) * (level)) + (Tilt_Ultimate_MinSpeed) * 64 - Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt) / 63;
        if(sp < (Tilt_Ultimate_MinSpeed))
        {
            sp = (Tilt_Ultimate_MinSpeed);
        }
        if(sp > Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt)
        {
            sp = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt;
        }
    }
    Velocity_Tilt = (level);
    Tilt_Softstart = Velocity_Tilt - 20;
    Tilt_Softstart *= 5;
    return sp;
}
