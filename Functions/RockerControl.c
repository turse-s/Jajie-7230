#include "RockerControl.h"
#include "sys.h"
#include "TaskAnalyze.h"
#include "SpecialFunctions.h"
#include "TaskCAN.h"
///////////////////////////////////////////////////////
/*摇杆控制函数*/
//u8 can_SpeedMod_Cmd[8] = {0x0b,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int Rocker_Select(u16 Down_CMD,u16 Down_Data)
{
	if(Down_CMD == Direction_Stop ||
		 Down_CMD == Direction_Up ||
		 Down_CMD == Direction_Down ||
		 Down_CMD == Direction_Right ||
		 Down_CMD == Direction_Left ||
		 Down_CMD == Direction_Down_Right ||
		 Down_CMD == Direction_Down_Left ||
		 Down_CMD == Direction_Up_Right ||
		 Down_CMD == Direction_Up_Left)
	{
		  Pan_Motion_Mode = Rocker_Motion;
			Tilt_Motion_Mode = Rocker_Motion;		//垂直目标速度初始化
			Special_Function = Tour_Scan_Disable;
			Rocker_Control(Down_CMD,Down_Data);
		return 1;
	}
		return 0;
}
int rocker_mark;
void Rocker_Control(u16 Down_CMD,u16 Down_Data)
{
#ifdef FOC
				can_SpeedMod_Cmd[4]=0x03;//速度爬升模式
				CAN1_Send_Msg(0x140|1,0,0, 0,8,can_SpeedMod_Cmd,0);//pan
				CAN1_Send_Msg(0x140|2,0,0, 0,8,can_SpeedMod_Cmd,0);//tilt		
#endif
    Stay_Time = 0;                                                      //守望位计时重置
		if (StartML > 0)																										//如果开始学习
		{
				if(Down_CMD == Direction_Stop)
				{
						if (zoom_record == 0)
						{
								Tracking_Fuction_Recording(Down_CMD,Down_Data);
						}
						else if (zoom_record == 1)
						{
								Tracking[StartML - zoom_record].Zoom = Zoom_Position;                
						}
				}
				else
				{
						if (StartML > 0)
						{
								Tracking_Fuction_Recording(Down_CMD,Down_Data);
						}
				}					
		}
    switch(Down_CMD)
    {
    case Direction_Stop:                                    
        Pan_Target_Speed = 0;                                               //水平目标速度初始化
        Tilt_Target_Speed = 0;
        break;
    case Direction_Up:
            Tilt_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1: Speed_Level_Tilt(Down_Data,Self_Adaption);
		break;
    case Direction_Down:
            Tilt_Target_Speed = (PTZ_Inverted)? Speed_Level_Tilt(Down_Data,Self_Adaption): ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
		break;
    case Direction_Left:
            Pan_Target_Speed = (PTZ_Inverted)? Speed_Level_Pan(Down_Data,Self_Adaption): ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
		break;
    case Direction_Right:
            Pan_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1: Speed_Level_Pan(Down_Data,Self_Adaption);
		break;
    case Direction_Up_Left:
            Tilt_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1: Speed_Level_Tilt(Down_Data,Self_Adaption);
            Pan_Target_Speed = (PTZ_Inverted)? Speed_Level_Pan(Down_Data,Self_Adaption): ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
		break;
    case Direction_Up_Right:
            Tilt_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1: Speed_Level_Tilt(Down_Data,Self_Adaption);
            Pan_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1: Speed_Level_Pan(Down_Data,Self_Adaption);
		break;
    case Direction_Down_Left:
            Tilt_Target_Speed = (PTZ_Inverted)? Speed_Level_Tilt(Down_Data,Self_Adaption): ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
            Pan_Target_Speed = (PTZ_Inverted)? Speed_Level_Pan(Down_Data,Self_Adaption): ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
		break;
    case Direction_Down_Right:
            Tilt_Target_Speed = (PTZ_Inverted)? Speed_Level_Tilt(Down_Data,Self_Adaption): ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
            Pan_Target_Speed = (PTZ_Inverted)? ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1: Speed_Level_Pan(Down_Data,Self_Adaption);
		break; 
    default:break;
				}
}

// 通用速度计算函数
int CalculateSpeed(u16 speed, u8 state_Self_Adaption, int ultimate_min, int ultimate_max, int scale)
{
    int sp = 0;
    u16 level = 0;
    u16 level_max = 0;

    // 提取速度等级
    level = speed & 0xff;

    if(state_Self_Adaption)
    {
        // 限制变焦位置
        if(Zoom_Position > Zoom_Position_Max)
        {
            Zoom_Position = Zoom_Position_Max;
        }
        if(Zoom_Position < Zoom_Position_Min)
        {
            Zoom_Position = Zoom_Position_Min;
        }

        // 计算速度等级最大值
        level_max = (Zoom_Position_Max - Zoom_Position) * 64 / (Zoom_Position_Max - Zoom_Position_Min);
        if(level_max < 3)
        {
            level_max = 3;
        }
        if(level_max > 64)
        {
            level_max = 64;
        }

        // 限制速度等级
        if (level > level_max)
        {
            level = level_max;
        }
    }

    if((level) <= 64 && (level) != 0)
    {
        // 计算最终速度
        sp = (((ultimate_max * scale - ultimate_min) * level) + ultimate_min * scale * 64 - ultimate_max) / 63;

        // 限制速度
        if(sp < ultimate_min)
        {
            sp = ultimate_min;
        }
        if(sp > ultimate_max * scale)
        {
            sp = ultimate_max * scale;
        }
    }

    return sp;
}

// 水平速度计算函数
int Speed_Level_Pan(u16 speed,u8 state_Self_Adaption)
{
    int sp = CalculateSpeed(speed>>8, state_Self_Adaption, Pan_Ultimate_MinSpeed, Pan_Ultimate_MaxSpeed, Speed_Scale_Pan);
    Velocity_Pan = speed >> 8;
    Pan_Softstart = Velocity_Pan - 20;
    Pan_Softstart *= 5;
#ifdef REVERSE_PAN
    return -sp;
#else
    return sp;
#endif
}

// 垂直速度计算函数
int Speed_Level_Tilt(u16 speed,u8 state_Self_Adaption)
{
    int sp = CalculateSpeed(speed, state_Self_Adaption, Tilt_Ultimate_MinSpeed, Tilt_Ultimate_MaxSpeed, Speed_Scale_Tilt);
    Velocity_Tilt = speed & 0xff;
    Tilt_Softstart = Velocity_Tilt - 20;
    Tilt_Softstart *= 5;
#ifdef REVERSE_TILT
    return -sp;
#else
    return sp;
#endif
}
