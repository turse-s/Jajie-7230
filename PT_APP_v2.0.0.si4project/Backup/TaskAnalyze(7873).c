#include "TaskAnalyze.h"
#include "TaskUSART3.h"
#include "TaskUSART_TX.h"
#include "24LCxx.h"
#include "hardware.h"
#include "stmflash.h"
#include "TaskEncoder.h"
#include "laser.h"
#include "CRC16.h"
double Pan_Angle = 0;
double Tilt_Angle = 0;
u8 PT_Position_Delete[1] = {0xff};
u8 Position_Data_Format[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 PT_Position_Record[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 PT_Position_Extract[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 ERASE_EE_Preset[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
u8 ERASE_EE_Parameter[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
//u8 aas = 0;
void Task_Analyze(void *p)
{
//    u16 i,j;
    OS_ERR err;
    p = p;
//    EEPRom_IIC_Init();
//    PWR_PVD_Init();
//    TIM10_Init(TIM10_ARR,TIM10_PSC);//1ms
//    Parameters_Init();
//    Sem_Analyze = OSSemCreate(1);
    OSSemCreate ((OS_SEM*   )&Sem_Analyze,
                 (CPU_CHAR* )"Sem_Analyze",
                 (OS_SEM_CTR)0,
                 (OS_ERR*   )&err);
    while(1)
    {
        OSSemPend(&Sem_Analyze,0,OS_OPT_PEND_BLOCKING,0,&err);
        Pend_Information_Function(&Analyze_BOX,Analyze_Function);
    }
}
u16 Counter1 = 0,Counter2 = 0;
void Analyze_Function(void)
{
    //Q_ERR err_q;
    u16 Down_CMD = 0;
    u32 Down_Data = 0;
    u16 i = 0,j = 0;
    u32 data_p = 0,data_t = 0;
    u8 send_len = 0;
    u8 send_data[20] = {0};
    u8 USART_x = 3;
    Down_CMD = Analyze_BOX.Memory_command[Analyze_BOX.extract_finger];
    Down_Data = Analyze_BOX.Memory_Data[Analyze_BOX.extract_finger];
    send_data[0] = 0xff;
    send_data[1] = PT_ID;
    send_data[2] = Down_CMD >> 8;
    send_data[3] = Down_CMD & 0xff;
    send_data[4] = Down_Data >> 8;
    send_data[5] = Down_Data & 0xff;
    send_len = 7;
    send_data[6] = Check_Sum(send_data,1,6);
    double TTA = 0;
    switch(Down_CMD)
    {
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//基本操作
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Direction_Stop:  
        Imitate_determine = 1;
        Pan_Motion_Mode = Rocker_Motion;                                    //设置水平摇杆模式
        Tilt_Motion_Mode = Rocker_Motion;                                   //设置垂直摇杆模式
        Pan_Target_Speed = 0;                                               //水平目标速度初始化
        Tilt_Target_Speed = 0;                                              //垂直目标速度初始化
        watcher_pause_state = watcher_continue;                 //守望位暂停状态改为‘继续’
        Special_Function = Tour_Scan_Disable;                               //停止巡检
        Stay_Time = 0;
        if(USART3_Protocol_Type == Protocol_VISCA)                          //判断协议是否为VISCA协议
        {
            memcpy(send_data,CAM_Zoom_Stop,6);                              //按协议内容填充
            send_len = 6;
        }
        USART_x = 3;//串口3
        break;
    case Direction_Up:
        if(!Axial)
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_TILT
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#endif
            Special_Function = Tour_Scan_Disable;                               //停止巡检
            Stay_Time = 0;                                                      //停留时间
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
    case Direction_Down:
        if(!Axial)
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_TILT
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#endif
            Special_Function = Tour_Scan_Disable;
            Stay_Time = 0;
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
    case Direction_Left:
        if(!Axial)                                                                  //判断是否为透传模式
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_PAN
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#else
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#endif
            Special_Function = Tour_Scan_Disable;
            Stay_Time = 0;
            arr1 = 1;
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
    case Direction_Right:
        if(!Axial)
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_PAN
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#else
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#endif
            Special_Function = Tour_Scan_Disable;
            Stay_Time = 0;
            arr1 = 1;
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
    case Direction_Up_Left:
        if(!Axial)
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_TILT
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#endif
#ifdef REVERSE_PAN
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#else
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#endif
            Special_Function = Tour_Scan_Disable;
            Stay_Time = 0;
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
    case Direction_Up_Right:
        if(!Axial)
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_TILT
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#else
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#endif
#ifdef REVERSE_PAN
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#else
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#endif
            Special_Function = Tour_Scan_Disable;
            Stay_Time = 0;
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
    case Direction_Down_Left:
        if(!Axial)
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_TILT
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#endif
#ifdef REVERSE_PAN
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#else
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#endif
            Special_Function = Tour_Scan_Disable;
            Stay_Time = 0;
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
    case Direction_Down_Right:
        if(!Axial)
        {
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
#ifdef REVERSE_TILT
            Tilt_Target_Speed = Speed_Level_Tilt(Down_Data,Self_Adaption);
#else
            Tilt_Target_Speed = ~Speed_Level_Tilt(Down_Data,Self_Adaption) + 1;
#endif
#ifdef REVERSE_PAN
            Pan_Target_Speed = ~Speed_Level_Pan(Down_Data,Self_Adaption) + 1;
#else
            Pan_Target_Speed = Speed_Level_Pan(Down_Data,Self_Adaption);
#endif
            Special_Function = Tour_Scan_Disable;
            Stay_Time = 0;
            USART_x = 0;//不发送
        }
        else
        {
            USART_x = 3;//串口3
        }
        break;
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//辅助开关
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case AUX_Switch_ON:
    {
        USART_x = 3;//串口3
    }
    break;
    case AUX_Switch_OFF:
    {
        USART_x = 3;//串口3
    }
    break;
    case ZOOM_Tele:
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            send_len = 6;
            memcpy(send_data,CAM_Zoom_Tele,send_len);
        }
        USART_x = 3;
        break;
    case ZOOM_Wide:
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            send_len = 6;
            memcpy(send_data,CAM_Zoom_Wide,send_len);
        }
        USART_x = 3;
        break;
    case FOCUS_Far:
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            send_len = 6;
            memcpy(send_data,CAM_Focus_Far,send_len);
        }
        USART_x = 3;
        break;
    case FOCUS_Near:
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            send_len = 6;
            memcpy(send_data,CAM_Focus_Near,send_len);
        }
        USART_x = 3;
        break;
//          激光灯操作
//      case Laser_ONOFF://             0x0101
//          memcpy(send_data,laser_onoff_cmd,send_len);
//          send_data[4] = Down_Data >> 8;
//          send_data[6] = Check_Sum(send_data,1,6);
//          USART_x = 2;
//      break;
//      case Laser_Current_Change://    0x0102
//          memcpy(send_data,laser_current_cmd,send_len);
//          send_data[4] = Down_Data >> 8;
//          send_data[6] = Check_Sum(send_data,1,6);
//          USART_x = 2;
//      break;
//      case Laser_Current_Set://       0x0103
//          memcpy(send_data,laser_current_set,send_len);
//          send_data[4] = Down_Data >> 8;
//          send_data[6] = Check_Sum(send_data,1,6);
//          USART_x = 2;
//      break;
//      case Laser_Step_Change://       0x0104
//          memcpy(send_data,laser_step_decrease,send_len);
//          send_data[4] = Down_Data >> 8;
//          send_data[5] = Down_Data & 0xff;
//          send_data[6] = Check_Sum(send_data,1,6);
//          USART_x = 1;
//      break;
//      case Laser_Position_Set://      0x0105
//          memcpy(send_data,laser_position_set,send_len);
//          send_data[4] = Down_Data >> 8;
//          send_data[5] = Down_Data & 0xff;
//          send_data[6] = Check_Sum(send_data,1,6);
//          USART_x = 2;
//      break;
//      case Laser_Reset://             0x0106
//          memcpy(send_data,laser_reset,send_len);
//          send_data[6] = Check_Sum(send_data,1,6);
//          USART_x = 1;
//        break;
    case IRIS_ON:
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            send_len = 6;
            memcpy(send_data,CAM_IRIS_UP,send_len);
        }
        USART_x = 3;
//          Encoder_Position_Storage();
        break;
    case IRIS_OFF:
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            send_len = 6;
            memcpy(send_data,CAM_IRIS_DOWN,send_len);
        }
//          STM32Sector_Erase(track_offect);
        USART_x = 3;
        break;
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//预置位功能
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Preset_Set:
        if(Down_Data >= 1 && Tilt_Hard_Limit_FLAG == Tilt_Limit_None)
        {
            //if((Down_Data < 200 && Down_Data != 105 && Down_Data != 111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))    //105,111,114,116为激光调节架的预置位由于不使用所以注释掉
            if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //取消掉了激光灯预置位，必要时可将上面的注释行恢复，并将此行注释
            {
                memcpy(PT_Position_Record,Position_Data_Format,Preset_Bytes_NUN);
                PT_Position_Record[8] = Zoom_Position >> 8;
                PT_Position_Record[9] = Zoom_Position & 0xff;
                PT_Position_Record[10] = (u32)(Pan_Real_Angle * 1000) >> 16;
                PT_Position_Record[11] = (u32)(Pan_Real_Angle * 1000) >> 8;
                PT_Position_Record[12] = (u32)(Pan_Real_Angle * 1000);
                PT_Position_Record[13] = (u32)(Tilt_Real_Angle * 1000) >> 16;
                PT_Position_Record[14] = (u32)(Tilt_Real_Angle * 1000) >> 8;
                PT_Position_Record[15] = (u32)(Tilt_Real_Angle * 1000);
                EEPRom_WriteDataLen(EEPROM_Preset,Down_Data,PT_Position_Record,Preset_Bytes_NUN);
            }
            else
            {
                Special_Preset_Set(Down_Data);
            }
        }
        USART_x = 3;
        break;
    case Preset_Delete:
        //if((Down_Data < 200 && Down_Data != 105 && Down_Data != 111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))
        if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //取消掉了激光灯预置位，必要时可将上面的注释行恢复，并将此行注释
        {
            EEPRom_WriteOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN,0xff);
        }
        else
        {
            Special_Preset_Delete(Down_Data);
        }
        USART_x = 3;
        break;
    case Preset_Extract:
        //if((Down_Data < 200 && Down_Data != 105 && Down_Data != 111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))
        if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //取消掉了激光灯预置位，必要时可将上面的注释行恢复，并将此行注释
        {
            Stay_Time = 0;
            Special_Function = Tour_Scan_Disable;
            if(EEPRom_ReadOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN) == 0xaa)
            {
                Pan_Motion_Mode = Location_Motion;
                Tilt_Motion_Mode = Location_Motion;
                EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,PT_Position_Extract,Preset_Bytes_NUN);
                Preset_Empty = 0;
                CAM_Zoom_Direct[4] = PT_Position_Extract[8] >> 4;
                CAM_Zoom_Direct[5] = PT_Position_Extract[8] & 0xf;
                CAM_Zoom_Direct[6] = PT_Position_Extract[9] >> 4;
                CAM_Zoom_Direct[7] = PT_Position_Extract[9] & 0xf;
                Zoom_Position = ((u16)PT_Position_Extract[8] << 8) + PT_Position_Extract[9];
                data_p = 0;
                data_p |= PT_Position_Extract[10];
                data_p <<= 8;
                data_p |= PT_Position_Extract[11];
                data_p <<= 8;
                data_p |= PT_Position_Extract[12];
                data_t = 0;
                data_t |= PT_Position_Extract[13];
                data_t <<= 8;
                data_t |= PT_Position_Extract[14];
                data_t <<= 8;
                data_t |= PT_Position_Extract[15];
                Pan_Target_Angle = ((double)data_p) / 1000.0;
                Tilt_Target_Angle = ((double)data_t) / 1000.0;
                if(Special_Function == Tour_Enable)
                {
                    Pan_Location_Speed = Tour_Speed_Pan;
                    Tilt_Location_Speed = Tour_Speed_Tilt;
                }
                else
                {
                    Pan_Location_Speed = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan;
                    Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt;
                }
                Place_InOut_Pan = OutPlace;
                Place_InOut_Tilt = OutPlace;
//                  Preset_Extract_Flag = 1;
//                  Preset_Extract_num = Down_Data;
#ifdef  ZS
                Extract_Number = Down_Data;
#endif
            }
            else
            {
                Preset_Empty = 1;
            }
        }
        else
        {
            Special_Preset_Extract(Down_Data);
        }
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
//              send_data[0] = 0x81;
//              send_data[1] = 0x01;
//              send_data[2] = 0x09;
//              send_data[3] = 0x52;
//              send_data[4] = 0x00;
//              send_data[5] = Down_Data >> 4 & 0x0f;
//              send_data[6] = Down_Data & 0x0f;
//              send_data[7] = 0xff;
            send_len = 9;
            memcpy(send_data,CAM_Zoom_Direct,send_len);
        }
        USART_x = 3;
        break;
    case Set_Angle_Pan:
        Stay_Time = 0;
        Special_Function = Tour_Scan_Disable;
        Pan_Motion_Mode = Location_Motion;
        if(Return_Data_Type == Return_Position)
        {
            Pan_Target_Angle = ((double)Down_Data) * Circle_Value / Pan_Encoder_Range + Pan_Origin_User;
        }
        else
        {
            Pan_Target_Angle = ((double)Down_Data) / Angle_Decimal + Pan_Origin_User;//浮点型
        }
        if(Pan_Target_Angle > Circle_Value)
        {
            Pan_Target_Angle = Pan_Target_Angle - Circle_Value;//浮点型
        }
        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;
        Place_InOut_Pan = OutPlace;
        USART_x = 0;
        break;
    case Set_Angle_Tilt:
        if(Return_Data_Type == Return_Position)
        {
            TTA = ((double)Down_Data) * Circle_Value / Tilt_Encoder_Range;
        }
        else
        {
            if(Tilt_Encoder_Direction == Encoder_Inversion)     //坐标反向处理
            {
                TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                if(TTA < 0)
                {
                    TTA = Circle_Value + TTA;//浮点型
                }
            }
            else
            {
                TTA = ((double)Down_Data) / Angle_Decimal + Tilt_Origin_User;//浮点型
                //TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                if(TTA > Circle_Value)
                {
                    TTA = TTA - Circle_Value;//浮点型
                }
            }
        }
        if(!(TTA > Tilt_Limit_Up && TTA < Tilt_Limit_Down))
        {
            Stay_Time = 0;
            Tilt_Target_Angle = TTA;        //将临时计算值传递给目标角度
            Special_Function = Tour_Scan_Disable;
            Tilt_Motion_Mode = Location_Motion;
            Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
            Place_InOut_Tilt = OutPlace;
        }
        USART_x = 0;
        break;
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//查询操作
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Query_Angle_Pan:
        Pan_Angle = pan_angle_query;
        send_data[2] = 0x00;
        send_data[3] = 0x59;
        if(Return_Data_Type == Return_Position)//编码器值回传
        {
            if(pan_encoder_type == Encoder_AS5600)
            {
                send_data[4] = ((u16)(Pan_Angle * Pan_Encoder_Range / Circle_Value)) >> 8;
                send_data[5] = ((u16)(Pan_Angle * Pan_Encoder_Range / Circle_Value)) & 0xff;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
            }
            else
            {
                send_data[4] = ((u32)(Pan_Angle * Pan_Encoder_Range / Circle_Value)) >> 16;
                send_data[5] = ((u32)(Pan_Angle * Pan_Encoder_Range / Circle_Value)) >> 8;
                send_data[6] = ((u32)(Pan_Angle * Pan_Encoder_Range / Circle_Value)) & 0xff;
                send_data[7] = Check_Sum(send_data,1,7);
                send_len = 8;
            }
        }
        else//角度回传
        {
            if(Angle_Accuracy_Flag)//高精度协议
            {
                send_data[4] = ((u32)(Pan_Angle * 1000)) >> 16;
                send_data[5] = ((u32)(Pan_Angle * 1000)) >> 8;
                send_data[6] = ((u32)(Pan_Angle * 1000)) & 0xff;
                send_data[7] = Check_Sum(send_data,1,7);
                send_len = 8;
            }
            else
            {
                send_data[4] = ((u16)(Pan_Angle * 100)) >> 8;
                send_data[5] = ((u16)(Pan_Angle * 100)) & 0xff;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
            }

#ifdef EB
            send_data[1] = 0x0b;
            Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);
            send_data[1] = 0x01;
#endif
//                #endif
        }
				#ifdef	ZS
						USART_x = 1;
				#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }

				#endif
        break;
    case Query_Angle_Tilt:
        Tilt_Angle = tilt_angle_query;
        send_data[2] = 0x00;
        send_data[3] = 0x5b;
        if(Return_Data_Type == Return_Position)//编码器值回传
        {
            if(tilt_encoder_type == Encoder_AS5600)
            {
                send_data[4] = ((u16)(Tilt_Angle * Tilt_Encoder_Range / Circle_Value)) >> 8;
                send_data[5] = ((u16)(Tilt_Angle * Tilt_Encoder_Range / Circle_Value)) & 0xff;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
            }
            else
            {
                send_data[4] = ((u32)(Tilt_Angle * Tilt_Encoder_Range / Circle_Value)) >> 16;
                send_data[5] = ((u32)(Tilt_Angle * Tilt_Encoder_Range / Circle_Value)) >> 8;
                send_data[6] = ((u32)(Tilt_Angle * Tilt_Encoder_Range / Circle_Value)) & 0xff;
                send_data[7] = Check_Sum(send_data,1,7);
                send_len = 8;
            }
        }
        else//角度回传
        {
            if(Angle_Accuracy_Flag)//高精度协议
            {
                send_data[4] = ((u32)(Tilt_Angle * 1000)) >> 16;
                send_data[5] = ((u16)(Tilt_Angle * 1000)) >> 8;
                send_data[6] = ((u16)(Tilt_Angle * 1000)) & 0xff;
                send_data[7] = Check_Sum(send_data,1,7);
                send_len = 8;
            }
            else
            {
                send_data[4] = ((u16)(Tilt_Angle * 100)) >> 8;
                send_data[5] = ((u16)(Tilt_Angle * 100)) & 0xff;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
            }
#ifdef EB
            send_data[1] = 0x0b;
            Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);
            send_data[1] = 0x01;
#endif
//                #endif
        }
				#ifdef	ZS
						USART_x = 1;
				#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }

				#endif
//            Post_Information_Function_TX(Sem_USART_TX,&USART_TX_BOX,USART_x,send_data,send_len);
        break;
    case Query_ZOOM:
        switch(Down_Data)
        {
        case 0:
            if(USART3_Protocol_Type == Protocol_VISCA)
            {
                send_len = 5;
                memcpy(send_data,CAM_ZoomPosInq,send_len);
                USART_x = 3;
            }
            else
            {
                send_data[2] = 0x00;
                send_data[3] = 0x5d;
                send_data[4] = Zoom_Position >> 8;
                send_data[5] = Zoom_Position & 0xff;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
                USART_x = 1;
            }
            break;
        case 1:
            if(USART3_Protocol_Type == Protocol_VISCA)
            {
                send_len = 5;
                memcpy(send_data,CAM_FocusPosInq,send_len);
                USART_x = 3;
            }
            else
            {
                send_data[2] = 0x00;
                send_data[3] = 0x65;
                send_data[4] = Focus_Position >> 8;
                send_data[5] = Focus_Position & 0xff;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
                USART_x = 1;
            }
            break;
        default:
            break;
        }
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
    case Query_ZOOM_Max:
    {
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            USART_x = 3;
        }
        else
        {
            send_data[2] = 0x01;
            send_data[3] = 0x69;
            send_data[4] = Zoom_Position_Max >> 8;
            send_data[5] = Zoom_Position_Max & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif 

        }
    }
    break;
    case Query_ZOOM_Min:
    {
        if(USART3_Protocol_Type == Protocol_VISCA)
        {
            USART_x = 3;
        }
        else
        {
            send_data[2] = 0x00;
            send_data[3] = 0x69;
            send_data[4] = Zoom_Position_Min >> 8;
            send_data[5] = Zoom_Position_Min & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif

        }
    }
    break;
    case Query_Versions_PT:
        send_data[2] = 0x28;
        send_data[3] = 0x10;
        send_data[4] = PT_Version >> 8;
        send_data[5] = PT_Version & 0xff;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
    case Query_Type_PT:
        send_data[2] = 0x28;
        send_data[3] = 0x1b;
        send_data[4] = PT_Type >> 8;
        send_data[5] = PT_Type & 0xff;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
    case Query_Time_Current:
        send_data[2] = 0x28;
        send_data[3] = 0x19;
        send_data[4] = TimeAndData >> 8;
        send_data[5] = TimeAndData & 0xff;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
    case Query_Encoder_PT:
        send_data[2] = 0x28;
        send_data[3] = 0x1a;
        send_data[4] = pan_encoder_type;
        send_data[5] = tilt_encoder_type;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
        USART_x = 1;
        break;
    case Query_Speed_Pan_Max:
        send_data[2] =0xa6;
        send_data[3] =0x15;
        send_data[4] = (Pan_A_MaxSpeed&0xff00)>>8;
        send_data[5] = Pan_A_MaxSpeed&0xff;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
    case Query_Speed_Pan_Min:
        send_data[2] =0xa6;
        send_data[3] =0x17;
        send_data[4] = (Pan_A_MinSpeed&0xff00)>>8;
        send_data[5] = Pan_A_MinSpeed&0xff;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
    case Query_Speed_Tilt_Max:
        send_data[2] =0xa6;
        send_data[3] =0x19;
        send_data[4] = (Tilt_A_MaxSpeed&0xff00)>>8;
        send_data[5] = Tilt_A_MaxSpeed&0xff;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
    case Query_Speed_Tilt_Min:
        send_data[2] =0xa6;
        send_data[3] =0x1b;
        send_data[4] = (Tilt_A_MinSpeed&0xff00)>>8;
        send_data[5] = Tilt_A_MinSpeed&0xff;
        send_data[6] = Check_Sum(send_data,1,6);
        send_len = 7;
#ifdef ZS
				 USART_x = 1;
#else
		if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 2;
                }
#endif
        break;
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//特殊功能
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Origin_Machine:            //设置机器原点
    {
        if (Down_Data == 0x0001)
        {
            memcpy(PT_Position_Record,Position_Data_Format,8);
            if(pan_encoder_type == 1)
            {
                //              PT_Position_Record[4] = (u16)(Pan_Real_Angle_1 * 100) >> 8;
                //              PT_Position_Record[5] = (u16)(Pan_Real_Angle_1 * 100) & 0xff;
                //              Pan_Machine_Origin_Angle = Pan_Real_Angle_1;
            }
            else if(pan_encoder_type == 2)
            {
                PT_Position_Record[4] = (Pan_Encoder_Data >> 8)  & 0xff;
                PT_Position_Record[5] = Pan_Encoder_Data & 0xff;
                Pan_Machine_Origin = Pan_Encoder_Data;
            }
            else if(pan_encoder_type == 3 || pan_encoder_type == 4)
            {
                PT_Position_Record[2] = (Pan_Encoder_Position_Real >> 16) & 0xff;
                PT_Position_Record[3] = (Pan_Encoder_Position_Real >> 8) & 0xff;
                PT_Position_Record[4] = Pan_Encoder_Position_Real & 0xff;
                Pan_Machine_Origin = Pan_Encoder_Position_Real;
                pan_track_offset = 0;
            }
            else
            {
                Pan_Machine_Origin = 0;
            }
            if(tilt_encoder_type == 1)
            {
                //              PT_Position_Record[6] = (u16)(Tilt_Real_Angle_1 * 100) >> 8;
                //              PT_Position_Record[7] = (u16)(Tilt_Real_Angle_1 * 100) & 0xff;
                //              Tilt_Machine_Origin_Angle = Tilt_Real_Angle_1;
                Origin_Machine_Flag = 1;
            }
            else if(tilt_encoder_type == 2)
            {
                PT_Position_Record[6] = (Tilt_Encoder_Data >> 8)  & 0xff;
                PT_Position_Record[7] = Tilt_Encoder_Data & 0xff;
                Tilt_Machine_Origin = Tilt_Encoder_Data;
                Origin_Machine_Flag = 5;
            }
            else if(tilt_encoder_type == 3 || tilt_encoder_type == 4)
            {
                PT_Position_Record[5] = (Tilt_Encoder_Position_Real >> 16) & 0xff;
                PT_Position_Record[6] = (Tilt_Encoder_Position_Real >> 8) & 0xff;
                PT_Position_Record[7] = Tilt_Encoder_Position_Real & 0xff;
                Tilt_Machine_Origin = Tilt_Encoder_Position_Real;
                Origin_Machine_Flag = 5;
                tilt_track_offset = 0;
            }
            else
            {
                Tilt_Machine_Origin = 0;
            }
            EEPRom_WriteDataLen(EEPROM_Parameter,Machine_Origin,PT_Position_Record,Parameter_Bytes_NUN);
            pan_data.Disposable_Switch = 1;
            tilt_data.Disposable_Switch = 1;
        }
    }
    break;
    case Clear_Parameter_Presets:            //清除用户参数和预置位
    {
        if (Down_Data == 0x0001)
        {
            Axial = 0;
            Return_Mode = Return_Query;
            USART1_DISABLE();
            USART2_DISABLE();
            USART3_DISABLE();
            for(i = 1; i < EE_TYPE / Preset_Bytes_NUN; i ++)
            {
                EEPRom_WriteDataLen(EEPROM_Preset,i,ERASE_EE_Preset,Preset_Bytes_NUN);
                //*****************************
                //喂狗
                IWDG_ReloadCounter();
            }
            for(j = 1; j < EE_TYPE / Parameter_Bytes_NUN; j ++)
            {
                EEPRom_WriteDataLen(EEPROM_Parameter,j,ERASE_EE_Parameter,Parameter_Bytes_NUN);
                //*****************************
                //喂狗
                IWDG_ReloadCounter();
            }
            delay_us(10000);
            SoftReset();
        }
    }
    break;
    case Clear_Flash:                //擦除FLSH
    {
        if (Down_Data == 0x0001)
        {
            STM32Sector_Erase(0x08020000);
        }
    }
    break;
    case Limit_Machine_Up:
        if(Down_Data == 0xffff)
        {
            send_data[2] = Limit_Machine_Up >> 8;
            send_data[3] = Limit_Machine_Up & 0xff;
            send_data[4] = (u16)(Tilt_Limit_Up * 100) >> 8;
            send_data[5] = (u16)(Tilt_Limit_Up * 100) & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
                if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 1;
                }
        }
        else
        {
            if(((double)Down_Data / 100) < Tilt_Angle_Limit_Up)
            {
                Tilt_Limit_Up = ((double)Down_Data) / 100;
                PT_Position_Record[1] = 0;
                PT_Position_Record[2] = 0;
                PT_Position_Record[3] = 0;
                PT_Position_Record[4] = 0;
                PT_Position_Record[5] = 0;
                PT_Position_Record[6] = Down_Data >> 8;
                PT_Position_Record[7] = Down_Data & 0xff;
                EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Up_EE,PT_Position_Record,Parameter_Bytes_NUN);
            }
            else
            {
                Tilt_Limit_Up = Tilt_Angle_Limit_Up;
                PT_Position_Record[1] = 0;
                PT_Position_Record[2] = 0;
                PT_Position_Record[3] = 0;
                PT_Position_Record[4] = 0;
                PT_Position_Record[5] = 0;
                PT_Position_Record[6] = (u16)(Tilt_Limit_Up * 100) >> 8;
                PT_Position_Record[7] = (u16)(Tilt_Limit_Up * 100) & 0xff;
                EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Up_EE,PT_Position_Record,Parameter_Bytes_NUN);
            }
        }
        break;
    case Limit_Machine_Down:
        if(Down_Data == 0xffff)
        {
            send_data[2] = Limit_Machine_Down >> 8;
            send_data[3] = Limit_Machine_Down &0xff;
            send_data[4] = (u16)(Tilt_Limit_Down * 100) >> 8;
            send_data[5] = (u16)(Tilt_Limit_Down * 100) & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
                if(Return_Mode == Return_Query_485)
                {
                    USART_x = 1;
                }
                else
                {
                    USART_x = 1;
                }
        }
        else
        {
            if((((double)Down_Data / 100) < 360) && (((double)Down_Data / 100) > Tilt_Angle_Limit_Down))
            {
                Tilt_Limit_Down = ((double)Down_Data) / 100;
                PT_Position_Record[1] = 0;
                PT_Position_Record[2] = 0;
                PT_Position_Record[3] = 0;
                PT_Position_Record[4] = 0;
                PT_Position_Record[5] = 0;
                PT_Position_Record[6] = Down_Data >> 8;
                PT_Position_Record[7] = Down_Data & 0xff;
                EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Down_EE,PT_Position_Record,Parameter_Bytes_NUN);
            }
            else
            {
                Tilt_Limit_Down = Tilt_Angle_Limit_Down;
                PT_Position_Record[1] = 0;
                PT_Position_Record[2] = 0;
                PT_Position_Record[3] = 0;
                PT_Position_Record[4] = 0;
                PT_Position_Record[5] = 0;
                PT_Position_Record[6] = (u16)(Tilt_Limit_Down * 100) >> 8;
                PT_Position_Record[7] = (u16)(Tilt_Limit_Down * 100) & 0xff;
                EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Down_EE,PT_Position_Record,Parameter_Bytes_NUN);
            }
        }
        break;
    case Tilt_Angle_Direction://俯仰反转指令
        if((Down_Data & 0xff) == 0xff)
        {
            send_data[5] = Tilt_Encoder_Direction;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
            if(Return_Mode == Return_Query_485)
            {
                USART_x = 1;
            }
            else
            {
                USART_x = 1;
            }
        }
        else
        {
            Tilt_Encoder_Direction = Down_Data;
            memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
            PT_Position_Record[7] = Tilt_Encoder_Direction;
            EEPRom_WriteDataLen(EEPROM_Parameter,Tilt_Angle_Direction_EE,PT_Position_Record,Parameter_Bytes_NUN);
        }
        break;
    case Set_PID_Pan://水平PID调试
        Pan_Kp = (Down_Data >> 8);
        Pan_Ki = (Down_Data & 0xff);
        PT_Position_Record[5] = Pan_Kd;
        PT_Position_Record[6] = Pan_Ki;
        PT_Position_Record[7] = Pan_Kp;
        EEPRom_WriteDataLen(EEPROM_Parameter, PID_Manual, PT_Position_Record, Parameter_Bytes_NUN);
        Pan_Kp = Pan_Kp / 100;
        Pan_Ki = Pan_Ki / 100;
        break;
    case Set_PID_Tilt://俯仰PID调试
        Tilt_Kp = (Down_Data >> 8);
        Tilt_Ki = (Down_Data & 0xff);
        PT_Position_Record[2] = Tilt_Kd;
        PT_Position_Record[3] = Tilt_Ki;
        PT_Position_Record[4] = Tilt_Kp;
        EEPRom_WriteDataLen(EEPROM_Parameter, PID_Manual, PT_Position_Record, Parameter_Bytes_NUN);
        Tilt_Kp =   Tilt_Kp / 1000;
        Tilt_Ki = Tilt_Ki / 1000;
        break;
    case Setup_Filter://滤波器设置
        if((Down_Data >> 8) == 0)
        {
            pan_data.Main_Switch = 0;
        }
        else
        {
            pan_data.Main_Switch = 1;
            pan_data.length = (Down_Data >> 8);
        }
        if((Down_Data & 0xff) == 0)
        {
            tilt_data.Main_Switch = 0;
        }
        else
        {
            tilt_data.Main_Switch = 1;
            tilt_data.length = (Down_Data & 0xff);
        }
        PT_Position_Record[4] = pan_data.Main_Switch;
        PT_Position_Record[5] = tilt_data.Main_Switch;
        PT_Position_Record[6] = pan_data.length;
        PT_Position_Record[7] = tilt_data.length;
        EEPRom_WriteDataLen(EEPROM_Parameter, Filter_EE, PT_Position_Record, Parameter_Bytes_NUN);
        break;
    case Stop_U2:
    {
        Pan_Motion_Mode = Rocker_Motion;                                    //设置水平摇杆模式
        Tilt_Motion_Mode = Rocker_Motion;                                   //设置垂直摇杆模式
        Pan_Target_Speed = 0;                                               //水平目标速度初始化
        Tilt_Target_Speed = 0;                                              //垂直目标速度初始化
        watcher_pause_state = watcher_continue;                 //守望位暂停状态改为‘继续’
        Special_Function = Tour_Scan_Disable;                               //停止巡检
        Stay_Time = 0;
        if(USART3_Protocol_Type == Protocol_VISCA)                          //判断协议是否为VISCA协议
        {
            memcpy(send_data,CAM_Zoom_Stop,6);                              //按协议内容填充
            send_len = 6;
        }
        USART_x = 3;//串口3
        U2_switch = (Down_Data & 0xff);
    }break;
    case Adaptation:                                                                                //电机速度自检
    {
		watcher_pause_state = watcher_pause;    //守望位暂停状态改为‘暂停’
        Pan_Motion_Mode = Rocker_Motion;
        Tilt_Motion_Mode = Rocker_Motion;
        Tilt_Target_Speed = 0;
        Pan_Target_Speed = 0;
        if(PWM_P == 0 && PWM_T == 0)
        {
            if (Down_Data  != 0)
            {
                Positioning_Sign = 0;
                Special_Function = Motor_Adaptation;
                Place_InOut_Tilt= OutPlace;
                send_data[2] = 0xa6;
                send_data[3] = 0x11;
                send_data[4] = 0x00;
                send_data[5] = 01;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
                USART_x = 1;
            }
            else
            {
                Pan_Ultimate_MaxSpeed = 4000;
                Tilt_Ultimate_MaxSpeed = 1000;
                Pan_Ultimate_MinSpeed = A_MinSpeed;
                Tilt_Ultimate_MinSpeed = A_MinSpeed;
                Scale_Velocity_Pan = 4.0;
                Scale_Velocity_Tilt = 1.1;
                Speed_Scale_Pan = 1.0;
                Speed_Scale_Tilt = 1.0;
                Speed_Set_Satus = 1;//速度设置标志位

                    PT_Position_Record[1] = (Pan_Ultimate_MaxSpeed >> 8)  & 0xff;
                    PT_Position_Record[2] = Pan_Ultimate_MaxSpeed & 0xff;
                    PT_Position_Record[3] = (Tilt_Ultimate_MaxSpeed >> 8)  & 0xff;
                    PT_Position_Record[4] = Tilt_Ultimate_MaxSpeed & 0xff;
                    PT_Position_Record[5] = (Pan_Ultimate_MinSpeed >> 8)  & 0xff;
                    PT_Position_Record[6] = Pan_Ultimate_MinSpeed & 0xff;
                    PT_Position_Record[7] = (Tilt_Ultimate_MinSpeed >> 8)  & 0xff;
                    PT_Position_Record[8] = Tilt_Ultimate_MinSpeed & 0xff;
                    PT_Position_Record[9] = ((u16)(Scale_Velocity_Pan * 100) >> 8) & 0xff;
										PT_Position_Record[10] = (u16)(Scale_Velocity_Pan * 100) & 0xff;
                    PT_Position_Record[11] = ((u16)(Scale_Velocity_Tilt * 100) >> 8) & 0xff;
										PT_Position_Record[12] = (u16)(Scale_Velocity_Tilt * 100) & 0xff;
                    PT_Position_Record[13] = (u8)(Speed_Scale_Pan * 100);
                    PT_Position_Record[14] = (u8)(Speed_Scale_Tilt * 100);
                    PT_Position_Record[15] = Speed_Set_Satus;
                EEPRom_WriteDataLen(EEPROM_Parameter,Max_Speed,PT_Position_Record,Preset_Bytes_NUN);
            }        
         }
    }break;
    case Speed_Set:                                                                                 //速度比例设置
    {
        Speed_Scale_Pan = ((Down_Data >> 8) > 100) ? 1.0f : (float)(Down_Data >> 8) / 100.0f;
        Speed_Scale_Tilt = ((Down_Data & 0xff) > 100) ? 1.0f : (float)(Down_Data & 0xff) / 100.0f;
        PT_Position_Record[11] = ((Down_Data >> 8) > 100) ? 0x64 : (float)(Down_Data >> 8);
        PT_Position_Record[12] = ((Down_Data & 0xff) > 100) ? 0x64 : (float)(Down_Data & 0xff);
        EEPRom_WriteDataLen(EEPROM_Parameter,(Max_Speed),PT_Position_Record,Preset_Bytes_NUN);
    }
    break;
#ifdef ZS
    case Extract_Number_Require://查询预置位就位情况
    
            if (Extract_Number)
            {
                send_len = 5;
                memcpy(send_data,Position_Arrival,send_len);
                send_data[3] = Extract_Number;
                USART_x = 3;
                Extract_Number = 0;
            }            
    break;
    case Return_Intime_Visca://visca实时回传
    
            //Pan_Current_Angle_User;
            Tilt_Current_Angle_User = 360 - Tilt_Current_Angle_User;
            send_data[0] = 0x81;
            send_data[1] = 0x01;
            send_data[2] = 0x09;
            send_data[3] = 0x79;
            
            send_data[4] = ((u16)(Pan_Current_Angle_User * 100)) >> 12;
            send_data[5] = (((u16)(Pan_Current_Angle_User * 100)) >> 8) &0x0f;
            send_data[6] = (((u16)(Pan_Current_Angle_User * 100)) &0xff) >> 4;
            send_data[7] = (((u16)(Pan_Current_Angle_User * 100)) &0xff)&0x0f;  //水平四位单独
            
            send_data[8] = ((u16)(Tilt_Current_Angle_User * 100)) >> 12;
            send_data[9] = (((u16)(Tilt_Current_Angle_User * 100)) >> 8) &0x0f;
            send_data[10] = (((u16)(Tilt_Current_Angle_User * 100)) &0xff) >> 4;
            send_data[11] = (((u16)(Tilt_Current_Angle_User * 100)) &0xff)&0x0f; //垂直四位单独
            
            send_data[12] = 0xFF;
            send_len = 13;
            USART_x = 3;
    break;

#endif
#ifdef SH
    case Left_Stay_Time:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id > 0 && scan_id < 6)
        {
						Sweep[scan_id].time_left = Down_Data >> 8;            
            PT_Position_Record[1] = Sweep[scan_id].time_left;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        }
    }
    break;
    case Right_Stay_Time:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id > 0 && scan_id < 6)
        {
						Sweep[scan_id].time_right = Down_Data >> 8;
            PT_Position_Record[2] = Sweep[scan_id].time_right;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        }
    }
    break;
    case Scan_Speed_Set:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id > 0 && scan_id < 6)
        {
            Sweep[scan_id].speed = Speed_Level_Pan(Down_Data,Self_Adaption);
            PT_Position_Record[3] = Sweep[scan_id].speed;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        }
    }
    break;
    case Scan_Border_Left:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id > 0 && scan_id < 6)
        {
						Sweep[scan_id].Zoom_Position_Left = Zoom_Position;
            Sweep[scan_id].border_left = Pan_Real_Angle;
            PT_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
            PT_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
            PT_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
            PT_Position_Record[10] = Zoom_Position >> 8;
            PT_Position_Record[11] = Zoom_Position & 0xff;																		
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        }
				USART_x = 0;
    }
    break;
    case Scan_Border_Right:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id > 0 && scan_id < 6)
        {
						Sweep[scan_id].Zoom_Position_Right = Zoom_Position;
            Sweep[scan_id].border_right = Pan_Real_Angle;
            PT_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
            PT_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
            PT_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
            PT_Position_Record[12] =  Zoom_Position >> 8;
            PT_Position_Record[13] = Zoom_Position & 0xff;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        }
				USART_x = 0;
    }
    break;
    case Start_Scan:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if(EEPRom_ReadOneByte(EEPROM_Parameter,(scan_id + 230 - 1) * Preset_Bytes_NUN) == 0xaa)
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Extract,Preset_Bytes_NUN);
            Sweep[scan_id].border_right = (double)(((PT_Position_Extract[7] | 0x0000) << 16) + ((PT_Position_Extract[8] | 0x0000) << 8) + PT_Position_Extract[9]) / 1000.0;
            Sweep[scan_id].border_left = (double)(((PT_Position_Extract[4] | 0x0000) << 16) + ((PT_Position_Extract[5] | 0x0000) << 8) + PT_Position_Extract[6]) / 1000.0;
            Sweep[scan_id].speed = PT_Position_Extract[3];
            Sweep[scan_id].time_left = PT_Position_Extract[1];
            Sweep[scan_id].time_right = PT_Position_Extract[2];
						Sweep[scan_id].Zoom_Position_Left = (PT_Position_Extract[10]  << 8) + PT_Position_Extract[11];
						Sweep[scan_id].Zoom_Position_Right = (PT_Position_Extract[12] << 8) + PT_Position_Extract[13];
            if (Sweep[scan_id].speed != 0)
            {
                Scan_Speed = Sweep[scan_id].speed;
            }
            else
            {
                Scan_Speed = Scan_Speed_Mid;
            }
						//储存坐标
            Section_Border1_Angle = ((double)Sweep[scan_id].border_left);
            Section_Border2_Angle = ((double)Sweep[scan_id].border_right);
						//时间
            Scan_Time_Left = Sweep[scan_id].time_left * 100;
            Scan_Time_Right = Sweep[scan_id].time_right * 100;
						//zoom值
						Scan_Zoom_Left = Sweep[scan_id].Zoom_Position_Left;
						Scan_Zoom_Right = Sweep[scan_id].Zoom_Position_Right;
            Special_Function = Scan_Enable;
            Pan_Location_Speed = Scan_Speed;
            Scan_Num = 1;
            Pan_Motion_Mode = Location_Motion;
            Place_InOut_Pan = OutPlace;
        }
        else
        {
            Pan_Motion_Mode = Rocker_Motion;
            Pan_Target_Speed = Scan_Speed;
        }
        USART_x = 0;
    }
    break;
    case Delete_Scan:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        Pan_Motion_Mode = Rocker_Motion;
        Pan_Target_Angle = Pan_Real_Angle;
        Pan_Target_Speed = 0;
        Sweep[scan_id].border_left = 0;
        Sweep[scan_id].border_right = 0;
        Sweep[scan_id].speed = 0;
        Sweep[scan_id].time_left = 0;
        Sweep[scan_id].border_right = 0;
			  Sweep[scan_id].Zoom_Position_Right = 0;
				Sweep[scan_id].Zoom_Position_Left = 0;
        PT_Position_Record[1] = Sweep[scan_id].time_left;
        PT_Position_Record[2] = Sweep[scan_id].time_right;
        PT_Position_Record[3] = Sweep[scan_id].speed;
        PT_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
        PT_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
        PT_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
        PT_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
        PT_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
        PT_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
			  PT_Position_Record[12] =  Zoom_Position >> 8;
        PT_Position_Record[13] = Zoom_Position & 0xff;
				PT_Position_Record[10] = Zoom_Position >> 8;
        PT_Position_Record[11] = Zoom_Position & 0xff;	
        EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        USART_x = 0;
    }
    break;
    case Rotate_Speed:
    {
        if ((Down_Data >> 8) <= 0x3f)
        {
            Speed_Rotate = Speed_Level_Pan(Down_Data,Self_Adaption);
        }
        USART_x = 0;
    }
    break;
    case Rotate_Start:
    {
        if(Speed_Rotate == 0)
        {
            Speed_Rotate = Pan_Ultimate_MaxSpeed/2;
        }
        switch(Down_Data >> 8)
        {
        case 0x01:
        {
            Special_Function = Rotating;
            Rotating_Direction = 0;
        }
        break;
        case 0x02:
        {
            Special_Function = Rotating;
            Rotating_Direction = 1;
        }
        break;
        default:
        {
            Pan_Target_Speed = 0;
            Special_Function = Tour_Scan_Disable;
        }
        break;
        }
        USART_x = 0;
    }
    break;
    case Patrol_Start:
    {
        if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000))
        {
            Special_Function = Tour_Scan_Disable;
            if(EEPRom_ReadOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN) == 0xaa)
            {
                EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,PT_Position_Extract,Preset_Bytes_NUN);
                data_p = 0;
                data_p |= PT_Position_Extract[10];
                data_p <<= 8;
                data_p |= PT_Position_Extract[11];
                data_p <<= 8;
                data_p |= PT_Position_Extract[12];
                data_t = 0;
                data_t |= PT_Position_Extract[13];
                data_t <<= 8;
                data_t |= PT_Position_Extract[14];
                data_t <<= 8;
                data_t |= PT_Position_Extract[15];
                Cruise.P_data_1 = ((double)data_p) / 1000.0;
                Cruise.T_data_1 = ((double)data_t) / 1000.0;
            }
        }
    }break;
    case Patrol_End:
    {
        if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000))
        {
            Special_Function = Tour_Scan_Disable;
            if(EEPRom_ReadOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN) == 0xaa)
            {
                EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,PT_Position_Extract,Preset_Bytes_NUN);
                data_p = 0;
                data_p |= PT_Position_Extract[10];
                data_p <<= 8;
                data_p |= PT_Position_Extract[11];
                data_p <<= 8;
                data_p |= PT_Position_Extract[12];
                data_t = 0;
                data_t |= PT_Position_Extract[13];
                data_t <<= 8;
                data_t |= PT_Position_Extract[14];
                data_t <<= 8;
                data_t |= PT_Position_Extract[15];
                Cruise.P_data_2 = ((double)data_p) / 1000.0;
                Cruise.T_data_2 = ((double)data_t) / 1000.0;
            }
        }
        USART_x = 0;
    }break;
    case Patrol_Setup:
    {
        Cruise.dir = Down_Data >> 8;
        Cruise.speed = Down_Data & 0xff;
        Cruise.speed_p = Speed_Level_Pan(Cruise.speed << 8,0);
        Cruise.speed_t = Speed_Level_Tilt(Cruise.speed,0);
        USART_x = 0;
    }break;
    case Patrol_Gradient:
    {
        Cruise.grads = Down_Data / 100.0;
        USART_x = 0;
    }break;
    case Patrol_Config:
    {
        int i1,i2,a[5];
			i2 = 0;
				if ((Down_Data  >> 8) == 0x01)
        {
            switch(Down_Data & 0xff)
            {
                case 0:
                    Special_Function = Skin_Fruit_ON;
                    Cruise.status = start;
                    Cruise.situation = 2;
                    break;
                case 1:
                    switch (Cruise.status)
                    {
                        case start://正在运行
                            Special_Function = Tour_Scan_Disable;
                            Cruise.status = pause;
														Cruise.T_data_pause = Tilt_Real_Angle;
														Cruise.P_data_pause = Pan_Real_Angle;
                            Cruise.Zoom_pause = Zoom_Position; 
														Pan_Motion_Mode = Rocker_Motion;                                    //璁剧疆姘村钩鎽囨潌妯″紡
														Tilt_Motion_Mode = Rocker_Motion;                                   //璁剧疆鍨傜洿鎽囨潌妯″紡
														Pan_Target_Speed = 0;                                               //姘村钩鐩爣閫熷害鍒濆鍖?
														Tilt_Target_Speed = 0;                                              //鍨傜洿鐩爣閫熷害鍒濆鍖
                        break;
                        case pause://暂停时
								//不予理睬 已经处于暂停模式				
                        break;
                        case recover:
                            Cruise.status = pause;
                        break;
                                    
                    }
                    break;
                case 3://恢复
                    switch (Cruise.status)
                    {
                        case start://正在运行
													for (i1 = 100;i1 <= 1000 ; i1++)
													{
														if (i1 % 10 == 9 && i1%9 == 8 && i1%8 ==7)
														{
																a[i2] = i1;
																i2++;
														}
													}
                                   //正在运行的时候恢复不起作用
                        break;
                        case pause://暂停时
                            Cruise.status = recover;
								                    Pan_Motion_Mode = Rocker_Motion;                                    //璁剧疆姘村钩鎽囨潌妯″紡
                            Tilt_Motion_Mode = Rocker_Motion;                                   //璁剧疆鍨傜洿鎽囨潌妯″紡
                            Pan_Target_Speed = 0;                                               //姘村钩鐩爣閫熷害鍒濆鍖?
                            Tilt_Target_Speed = 0;  
                            send_len = 9;
                            memcpy(send_data,CAM_Zoom_Direct,send_len);
                            send_data[4] = Cruise.Zoom_pause >> 12;
                            send_data[5] = (Cruise.Zoom_pause >> 8) &0x0f;
                            send_data[6] = (Cruise.Zoom_pause &0xff) >> 4;
                            send_data[7] = (Cruise.Zoom_pause &0xff) &0x0f;  //zoom set
                            Special_Function = Skin_Fruit_Pause;
                            USART_x = 3;//不予理睬 已经处于暂停模式				
                        break;
                        default:break;                    }
                break;
                default:
                    Special_Function = Tour_Scan_Disable;
								    Pan_Motion_Mode = Rocker_Motion;                                    //璁剧疆姘村钩鎽囨潌妯″紡
										Tilt_Motion_Mode = Rocker_Motion;                                   //璁剧疆鍨傜洿鎽囨潌妯″紡
										Pan_Target_Speed = 0;                                               //姘村钩鐩爣閫熷害鍒濆鍖?
										Tilt_Target_Speed = 0;                                              //鍨傜洿鐩爣閫熷害鍒濆鍖?
										break;
            }break;
        }
    }break;
    
#endif
#ifdef HY
		case Scan_Border_Left:
		{
				u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id < 6)
        {
						Sweep[scan_id].Zoom_Position_Left = Zoom_Position;
            Sweep[scan_id].border_left = Pan_Real_Angle;
            PT_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
            PT_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
            PT_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
            PT_Position_Record[10] = Zoom_Position >> 8;
            PT_Position_Record[11] = Zoom_Position & 0xff;																		
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        }
				USART_x = 0;		
		}break;
		case Scan_Border_Right:
		{
				u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id < 6)
        {
						Sweep[scan_id].Zoom_Position_Right = Zoom_Position;
            Sweep[scan_id].border_right = Pan_Real_Angle;
            PT_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
            PT_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
            PT_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
            PT_Position_Record[12] =  Zoom_Position >> 8;
            PT_Position_Record[13] = Zoom_Position & 0xff;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
        }
				USART_x = 0;
		}break;
		//开始线扫
		case Start_Scan:
		{
				u16 scan_id;
        scan_id = (Down_Data & 0xff);
				//先判断id是否合理，如果不合理则直接退出
        if (scan_id < 6)
        {
					//先设置速度，在去执行id
					//如果没有刻意设置速度，则为默认速度0F
					if(Down_Data & 0xff00)
					{
						Sweep[scan_id].speed = Speed_Level_Pan(0x0F00,Self_Adaption);
						PT_Position_Record[3] = Sweep[scan_id].speed;
						EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
					
					}
					else //进入则说明有速度设置，将data中的速度进行存储
					{
					  Sweep[scan_id].speed = Speed_Level_Pan(Down_Data,Self_Adaption);
            PT_Position_Record[3] = Sweep[scan_id].speed;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
					}
					//执行id
					if(EEPRom_ReadOneByte(EEPROM_Parameter,(scan_id + 230 - 1) * Preset_Bytes_NUN) == 0xaa)
					{
            EEPRom_ReadDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Extract,Preset_Bytes_NUN);
            Sweep[scan_id].border_right = (double)(((PT_Position_Extract[7] | 0x0000) << 16) + ((PT_Position_Extract[8] | 0x0000) << 8) + PT_Position_Extract[9]) / 1000.0;
            Sweep[scan_id].border_left = (double)(((PT_Position_Extract[4] | 0x0000) << 16) + ((PT_Position_Extract[5] | 0x0000) << 8) + PT_Position_Extract[6]) / 1000.0;
            Sweep[scan_id].speed = PT_Position_Extract[3];
            Sweep[scan_id].time_left = PT_Position_Extract[1];
            Sweep[scan_id].time_right = PT_Position_Extract[2];
						Sweep[scan_id].Zoom_Position_Left = (PT_Position_Extract[10]  << 8) + PT_Position_Extract[11];
						Sweep[scan_id].Zoom_Position_Right = (PT_Position_Extract[12] << 8) + PT_Position_Extract[13];
            if (Sweep[scan_id].speed != 0)
            {
                Scan_Speed = Sweep[scan_id].speed;
            }
            else
            {
                Scan_Speed = Scan_Speed_Mid;
            }
						//储存坐标
            Section_Border1_Angle = ((double)Sweep[scan_id].border_left);
            Section_Border2_Angle = ((double)Sweep[scan_id].border_right);
						//时间
            Scan_Time_Left = Sweep[scan_id].time_left * 100;
            Scan_Time_Right = Sweep[scan_id].time_right * 100;
						//zoom值
						Scan_Zoom_Left = Sweep[scan_id].Zoom_Position_Left;
						Scan_Zoom_Right = Sweep[scan_id].Zoom_Position_Right;
            Special_Function = Scan_Enable;
            Pan_Location_Speed = Scan_Speed;
            Scan_Num = 1;
            Pan_Motion_Mode = Location_Motion;
            Place_InOut_Pan = OutPlace;
					}
					else
					{
            Pan_Motion_Mode = Rocker_Motion;
            Pan_Target_Speed = Scan_Speed;
					}
				}
		}break;
		case Delete_Scan:
		{
				u16 scan_id;
        scan_id = (Down_Data & 0xff);
        Pan_Motion_Mode = Rocker_Motion;
        Pan_Target_Angle = Pan_Real_Angle;
        Pan_Target_Speed = 0;
        Sweep[scan_id].border_left = 0;
        Sweep[scan_id].border_right = 0;
        Sweep[scan_id].speed = 0;
        Sweep[scan_id].time_left = 0;
        Sweep[scan_id].border_right = 0;
			  Sweep[scan_id].Zoom_Position_Right = 0;
				Sweep[scan_id].Zoom_Position_Left = 0;
        PT_Position_Record[1] = Sweep[scan_id].time_left ;
        PT_Position_Record[2] = Sweep[scan_id].time_right ;
        PT_Position_Record[3] = Sweep[scan_id].speed;
        PT_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
        PT_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
        PT_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
        PT_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
        PT_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
        PT_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
			  PT_Position_Record[12] =  Zoom_Position >> 8;
        PT_Position_Record[13] = Zoom_Position & 0xff;
				PT_Position_Record[10] = Zoom_Position >> 8;
        PT_Position_Record[11] = Zoom_Position & 0xff;	
        EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
		}break;
		//逆时针水平旋转开始
		case Rotate_Contrary_Start:
		{	
				//旋转前判断速度
				if(Speed_Rotate == 0)
        {
            Speed_Rotate = Pan_Ultimate_MaxSpeed/2;
        }
				Special_Function = Rotating;
        Leading_mod = 1;
		
		}break;
		//学习功能记录
		case Imitate_Record:
		{
			for(Imi.Imitation_top=0;Imi.Imitation_top < Imitate_size;Imi.Imitation_top++)
			{
				Imi.Instruct_Data[Imi.Imitation_top] = 0;
			}
			//首先记录当前位置坐标，填充goto命令
			Imi.Imitation_mode = Imitate_ON;     //将模仿开关打开
			//填充goto命令，保存开始坐标
      Imi.Instruct_Data[0] =  0x344D;								//cmd
			Imi.Instruct_Data[1] =  Tilt_Real_Angle*100;			//data
			Imi.Instruct_Data[2] =  0;										//time
			Imi.Instruct_Data[3] =  0x344B;
			Imi.Instruct_Data[4] =  Pan_Real_Angle*100;
			Imi.Instruct_Data[5] =  0;
			Imi.Instruct_Data[6] =  0;
			Imi.Instruct_Data[7] =  0;
			Imi.Instruct_Data[8] =  0;
			Imi.Imitation_top=9;
			//可以开始学习了
		}break;
		case Imitate_Stop:
		{
			Imi.Imitation_mode = Imitate_OFF;     //将模仿开关关闭，保存刚刚数据填充
		}break;
		//学习路径执行
		case Imitate_start:
		{
			Imi.Imitation_mode = Route_ON;
			Imi.Imitation_Statt_Mode=0;
			for(Imi.Imitation_top=0;Imi.Imitation_top < Imitate_size;Imi.Imitation_top++)
			{
				Imi.Instruct_Data[Imi.Imitation_top] = 0;
			}
		}
		//水果皮功能
		case Skin_Direction_Set:
		{
			//获取id号
			Fruit.Id = Down_Data & 0xFF;
			//获取方向
			Fruit.Direction = Down_Data >>8;
			//存E方中
			PT_Position_Record[1] = Fruit.Id;
			PT_Position_Record[2] = Fruit.Direction;
			EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),PT_Position_Record,Preset_Bytes_NUN);
		}break;
		case Skin_UP_Set:
		{
			//获取id号
			Fruit.Id = Down_Data & 0xFF;
			//保存当前垂直坐标，存为上限位
			Fruit.Up = Tilt_Real_Angle;
			PT_Position_Record[3] = (u32)(Fruit.Up * 1000) >> 16;
      PT_Position_Record[4] = (u32)(Fruit.Up * 1000) >> 8;
      PT_Position_Record[5] = (u32)(Fruit.Up * 1000);
			EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),PT_Position_Record,Preset_Bytes_NUN);
		}break;
		case Skin_Down_Set:
		{
			//获取id号
			Fruit.Id = Down_Data & 0xFF;
			Fruit.Down = Tilt_Real_Angle;
			PT_Position_Record[6] = (u32)(Fruit.Down * 1000) >> 16;
      PT_Position_Record[7] = (u32)(Fruit.Down * 1000) >> 8;
      PT_Position_Record[8] = (u32)(Fruit.Down * 1000);
			EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),PT_Position_Record,Preset_Bytes_NUN);
		}
#endif
        case FOV_Config:
        {
            FOV = Down_Data / 100.0;
        }break;
//        case 0xff01://更改串口1波特率
//        {
//            u8 aa = 0,bc[8]= {0};
//            USART1_DISABLE();
//            USART1_BAND = Down_Data;
//            memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
//            PT_Position_Record[5] = (Down_Data >> 16) & 0xff;
//            PT_Position_Record[6] = (Down_Data >> 8) & 0xff;
//            PT_Position_Record[7] = Down_Data & 0xff;
//            EEPRom_WriteDataLen(EEPROM_Parameter,USART1_BAUDRATE,PT_Position_Record,Parameter_Bytes_NUN);
//            USART1_INT_Init(USART1_BAND);
//            aa = EEPRom_ReadOneByte(EEPROM_Parameter,(USART1_BAUDRATE - 1) * Parameter_Bytes_NUN);
//            EEPRom_ReadDataLen(EEPROM_Parameter,USART1_BAUDRATE,bc,Parameter_Bytes_NUN);
//        }break;
//        case 0xff02://更改串口2波特率
//        {
//            USART2_DISABLE();
//            USART2_BAND = Down_Data;
//            memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
//            PT_Position_Record[5] = (Down_Data >> 16) & 0xff;
//            PT_Position_Record[6] = (Down_Data >> 8) & 0xff;
//            PT_Position_Record[7] = Down_Data & 0xff;
//            EEPRom_WriteDataLen(EEPROM_Parameter,USART2_BAUDRATE,PT_Position_Record,Parameter_Bytes_NUN);
//            USART2_INT_Init(USART2_BAND);
//        }break;
//        case 0xff03://更改串口3波特率
//        {
//            USART3_DISABLE();
//            USART3_BAND = Down_Data;
//            memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
//            PT_Position_Record[5] = (Down_Data >> 16) & 0xff;
//            PT_Position_Record[6] = (Down_Data >> 8) & 0xff;
//            PT_Position_Record[7] = Down_Data & 0xff;
//            EEPRom_WriteDataLen(EEPROM_Parameter,USART3_BAUDRATE,PT_Position_Record,Parameter_Bytes_NUN);
//            USART3_INT_Init(USART3_BAND);
//        }break;
    default:
        switch(Down_CMD & 0xff)
        {
        case Set_Angle_Pan:
            Stay_Time = 0;
            Special_Function = Tour_Scan_Disable;
            Pan_Motion_Mode = Location_Motion;
            Pan_Target_Angle = ((double)Down_Data) / Angle_Decimal + Pan_Origin_User;//浮点型
            if(Pan_Target_Angle > 360.0)
            {
                Pan_Target_Angle = Pan_Target_Angle - 360.0;//浮点型
            }
            Pan_Location_Speed = Speed_Level_Pan(Down_CMD & 0xff00,0);
            Place_InOut_Pan = OutPlace;
#ifdef ZS
                Extract_Number = 0;
                if (Place_InOut_Pan == InPlace)
                {
                        send_data[0] = 0xFF;
                        send_data[1] = 0x01;
                        send_data[2] = 0x66;
                        send_data[3] = 0x4b;
                        send_data[4] = 0x00;
                        send_data[5] = 0x01;
                        send_data[6] = Check_Sum(send_data,1,6);
                        send_len = 7;
                        			if(Return_Mode == Return_Query_485)
			{
				USART_x = 1;
			}
			else
			{
				USART_x = 2;
			}
                }
#endif
            break;
        case Set_Angle_Tilt:
            if(Return_Data_Type == Return_Position)
            {
                TTA = ((double)Down_Data) * Circle_Value / Tilt_Encoder_Range;  //Down_Data
            }
            else
            {
                if(Tilt_Encoder_Direction == Encoder_Inversion)     //坐标反向处理
                {
                    TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                    if(TTA < 0)
                    {
                        TTA = Circle_Value + TTA;//浮点型
                    }
                }
                else
                {
                    TTA = ((double)Down_Data) / Angle_Decimal + Tilt_Origin_User;//浮点型  zxz 20220927 降该行去掉改为下一行 TTA = ((double)Down_Data) / Angle_Decimal;
                    //TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                    if(TTA > Circle_Value)
                    {
                        TTA = TTA - Circle_Value;//浮点型
                    }
                }
            }
            if(!(TTA > Tilt_Limit_Up && TTA < Tilt_Limit_Down))
            {
                Stay_Time = 0;
                Tilt_Target_Angle = TTA;
                Special_Function = Tour_Scan_Disable;
                Tilt_Motion_Mode = Location_Motion;
                Tilt_Location_Speed = Speed_Level_Tilt((Down_CMD >> 8) & 0xff,0);
                Place_InOut_Tilt = OutPlace;
            }
#ifdef ZS
                Extract_Number = 0;
                if (Place_InOut_Tilt == InPlace)
                {
                        send_data[0] = 0xFF;
                        send_data[1] = 0x01;
                        send_data[2] = 0x66;
                        send_data[3] = 0x4d;
                        send_data[4] = 0x00;
                        send_data[5] = 0x01;
                        send_data[6] = Check_Sum(send_data,1,6);
                        send_len = 7;
                  			if(Return_Mode == Return_Query_485)
			{
				USART_x = 1;
			}
			else
			{
				USART_x = 2;
			}
                }
#endif
            break;
#ifdef HY
                        case Scan_Stay_Time:
                            {
                                    u16 scan_id;
                                    scan_id = (Down_CMD & 0xff00)>>8;
                                    if (scan_id < 6)
                                    {
                                        Sweep[scan_id].time_left = Down_Data >> 8;
                                        Sweep[scan_id].time_right = Down_Data & 0xff;                               
                                        PT_Position_Record[1] = Sweep[scan_id].time_left ;
                                        PT_Position_Record[2] = Sweep[scan_id].time_right ;
                                        EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
                                    }
                            }break;
                        case Skin_Start:
                        {
                            //获取执行id
                            Fruit.Id = Down_CMD >> 8;
                            //获取速度
#ifdef REVERSE_TILT
                    Fruit.Speed = ~Speed_Level_Tilt(Down_Data>>8,Self_Adaption) + 1;
#else
                    Fruit.Speed = Speed_Level_Tilt(Down_Data>>8,Self_Adaption);
#endif
                        //保存一圈垂直偏移角度
                            if((Down_Data &0xff) == 0x00)
                            {
                                Fruit.Tilt_Dif = 5;
                            }
                            else
                            {
                                Fruit.Tilt_Dif = Down_Data &0xff;
                            }
        
                        //提取e方中的数据，上下限位方向
                        if(EEPRom_ReadOneByte(EEPROM_Parameter,(Fruit.Id + Skin_Fruit -1  * Preset_Bytes_NUN) == 0xaa))
                            {
                    EEPRom_ReadDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),PT_Position_Extract,Preset_Bytes_NUN);
                              data_t = 0;
                    data_t |= PT_Position_Extract[3];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract[4];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract[5];
                    Fruit.Up = ((double)data_t) / 1000.0;
                                data_t = 0;
                    data_t |= PT_Position_Extract[6];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract[7];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract[8];
                    Fruit.Down = ((double)data_t) / 1000.0;
                                Fruit.Direction =   PT_Position_Extract[2];
                            }
                        Special_Function = Skin_Fruit_ON;
                        Fruit.Statt_Mode = 0;
                        Pan_Target_Speed = 0;
        
                        }break;
#endif

        case Preset_Extract:
            //if((Down_Data < 200 && Down_Data != 105 && Down_Data !=111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))
            if( ( Down_Data < 200 ) || ( Down_Data > 256 && Down_Data < 2000 ) )
            {
                Stay_Time = 0;
                if(EEPRom_ReadOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN) == 0xaa)
                {
                    EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,PT_Position_Extract,Preset_Bytes_NUN);
                    Preset_Empty = 0;
                    CAM_Zoom_Direct[4] = PT_Position_Extract[8] >> 4;
                    CAM_Zoom_Direct[5] = PT_Position_Extract[8] & 0xf;
                    CAM_Zoom_Direct[6] = PT_Position_Extract[9] >> 4;
                    CAM_Zoom_Direct[7] = PT_Position_Extract[9] & 0xf;
                    Zoom_Position = ((u16)PT_Position_Extract[8] << 8) + PT_Position_Extract[9];
                    data_p = 0;
                    data_p |= PT_Position_Extract[10];
                    data_p <<= 8;
                    data_p |= PT_Position_Extract[11];
                    data_p <<= 8;
                    data_p |= PT_Position_Extract[12];
                    data_t |= PT_Position_Extract[13];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract[14];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract[15];
                    Pan_Target_Angle = ((double)data_p) / 1000.0;
                    Tilt_Target_Angle = ((double)data_t) / 1000.0;
                    if(Special_Function == Tour_Enable)
                    {
                        Pan_Location_Speed = Tour_Speed_Pan;
                        Tilt_Location_Speed = Tour_Speed_Tilt;
                    }
                    else
                    {
                        Pan_Location_Speed = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan;
                        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt;
                    }
                    Pan_Motion_Mode = Location_Motion;
                    Tilt_Motion_Mode = Location_Motion;
                    Place_InOut_Pan = OutPlace;
                    Place_InOut_Tilt = OutPlace;
                    Preset_Extract_Flag = 1;
                    Preset_Extract_num = Down_Data;
#ifdef  ZS
                Extract_Number = Down_Data;
#endif
                }
                else
                {
                    Preset_Empty = 1;
                }
            }
            else
            {
                Special_Preset_Extract(Down_Data);
            }
            send_data[2] = 0x00;
            send_data[6] = Check_Sum(send_data,1,6);
            if(USART3_Protocol_Type == Protocol_VISCA)
            {
                send_len = 9;
                memcpy(send_data,CAM_Zoom_Direct,send_len);
            }
            USART_x = 3;
            break;
        }
        break;
    }
    //*****************************************
    if(USART_x)
    {
        if(USART_x == 1)
        {
            Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);
        }
        if(USART_x == 2)
        {
            Post_Information_Function_TX(Sem_USART_TX,&USART2_TX_BOX,USART_x,send_data,send_len);
        }
        if(USART_x == 3)
        {
            Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,USART_x,send_data,send_len);
        }
        if(USART3_Protocol_Type == Protocol_VISCA && Down_CMD == Direction_Stop)
        {
            memcpy(send_data,CAM_ZoomPosInq,5);
            send_len = 5;
            Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,USART_x,send_data,send_len);
        }
    }
    //*****************************************
}
//*************************************************************************
//特殊预置位
u8 PT_Scan1,PT_Scan2;
u8 Scan_Border1[8],Scan_Border2[8];
void Special_Preset_Set(u16 num)
{
    u16 i1 = 0;
    u32 data_p1 = 0,data_p2 = 0;
    switch(num)
    {
#ifdef ZS02
    case 103:
    {
        Pan_Angle_PowerBoot = Pan_Real_Angle;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Pan_Angle_PowerBoot * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Pan_Angle_PowerBoot * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Pan_Angle_PowerBoot * 1000) & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,103,PT_Position_Record,Parameter_Bytes_NUN);
    }break;
    case 104:
    {
        Tilt_Angle_PowerBoot = Tilt_Real_Angle;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Tilt_Angle_PowerBoot * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Tilt_Angle_PowerBoot * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Tilt_Angle_PowerBoot * 1000) & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,104,PT_Position_Record,Parameter_Bytes_NUN);
    }break;
#endif   
    case Origin_Pan:                  //201设置水平原点
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_Origin_User = Pan_Real_Angle;
        pan_data.Disposable_Switch = 1;
        EEPRom_WriteDataLen(EEPROM_Parameter,Origin_Pan,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Origin_Tilt:                 //202设置垂直原点
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        Tilt_Origin_User = Tilt_Real_Angle;
        tilt_data.Disposable_Switch = 1;
        EEPRom_WriteDataLen(EEPROM_Parameter,Origin_Tilt,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Angle_Return_Mode:           //203实时回传模式
    {
//            if(Return_Mode != Return_Query_485)
//            {
        Return_Mode = Return_RealTime;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record,Parameter_Bytes_NUN);
//            }
    }
    break;
    case Tour_Speed_Regulation:       //204巡检速度增加
    {
        if (Speed_Set_Satus == 0)
        {
            Tour_Speed_Pan = (Tour_Speed_Pan + Speed_Offset > Tour_Speed_Max) ? Tour_Speed_Max : Tour_Speed_Pan + Speed_Offset;
            Tour_Speed_Tilt = (Tour_Speed_Tilt + Speed_Offset > Tour_Speed_Max) ? Tour_Speed_Max : Tour_Speed_Tilt + Speed_Offset;
        }
        else
        {
            Tour_Speed_Pan = (Tour_Speed_Pan + Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 20 > Pan_Ultimate_MaxSpeed * Speed_Scale_Pan) ? Pan_Ultimate_MaxSpeed * Speed_Scale_Pan : Tour_Speed_Pan + Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 20;
            Tour_Speed_Tilt = (Tour_Speed_Tilt + Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt / 20 > Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt) ? Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt : Tour_Speed_Tilt + Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt / 20;
        }
				Pan_Location_Speed = Tour_Speed_Pan;
				Tilt_Location_Speed = Tour_Speed_Tilt;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[4] = Tour_Speed_Pan >> 8;
        PT_Position_Record[5] = Tour_Speed_Pan & 0xff;
        PT_Position_Record[6] = Tour_Speed_Tilt >> 8;
        PT_Position_Record[7] = Tour_Speed_Tilt & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_Speed_Level:            //205巡检速度最大
    {
        if (Speed_Set_Satus == 0)
        {
            Tour_Speed_Pan = Tour_Speed_Max;
            Tour_Speed_Tilt = Tour_Speed_Max;
        }
        else
        {
            Tour_Speed_Pan =  Pan_Ultimate_MaxSpeed * Speed_Scale_Pan;
            Tour_Speed_Tilt = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt;
        }
				Pan_Location_Speed = Tour_Speed_Pan;
				Tilt_Location_Speed = Tour_Speed_Tilt;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[4] = Tour_Speed_Pan >> 8;
        PT_Position_Record[5] = Tour_Speed_Pan & 0xff;
        PT_Position_Record[6] = Tour_Speed_Tilt >> 8;
        PT_Position_Record[7] = Tour_Speed_Tilt & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Regulation:    //206巡检停留时间增加
    {
        if(Tour_Time < Tour_WaitTime_Max)
        {
            Tour_Time = Tour_Time + WaitTime_Offset;
        }
        else
        {
            Tour_Time = Tour_WaitTime_Max;
        }
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Level:         //207巡检停留时间最大
    {
        Tour_Time = Tour_WaitTime_Max;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Regulation: //208守望停留时间增加
    {
        if(Watcher_Time < Watcher_WaitTime_Max)
        {
            Watcher_Time = Watcher_Time + WaitTime_Offset;
        }
        else
        {
            Watcher_Time = Watcher_WaitTime_Max;
        }
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Level:      //209守望停留时间最大
    {
        Watcher_Time = Watcher_WaitTime_Max;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Regulation:       //210扫描速度增加
    {
        if (Speed_Set_Satus == 0)
        {
            Scan_Speed = (Scan_Speed + Speed_Offset > Scan_Speed_Max) ? Scan_Speed_Max : Scan_Speed + Speed_Offset;
        }
        else
        {
            Scan_Speed = (Scan_Speed + Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 20 > Pan_Ultimate_MaxSpeed * Speed_Scale_Pan) ? Pan_Ultimate_MaxSpeed * Speed_Scale_Pan : Scan_Speed + Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 20;
        }
		Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Scan_Speed >> 8;
        PT_Position_Record[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Level:            //211扫描速度最大
    {
         if (Speed_Set_Satus == 0)
         {
               Scan_Speed = Scan_Speed_Max;
         }
         else       
         {
              Scan_Speed =  Pan_Ultimate_MaxSpeed * Speed_Scale_Pan;
         }
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Scan_Speed >> 8;
        PT_Position_Record[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Return_Type:                 //213角度回传
    {
        Return_Data_Type = Return_Angle;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Data_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,Return_Type,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
//        case Laser_Adjust_Switch_UNV:       //162退出激光灯调节模式,宇视
    case Laser_Adjust_Switch:         //214进入激光灯调节模式
    {
        Axial = 1;
        Pan_Motion_Mode = Rocker_Motion;
        Tilt_Motion_Mode = Rocker_Motion;
        Pan_Target_Speed = 0;
        Tilt_Target_Speed = 0;
        Special_Function = Tour_Scan_Disable;
    }
    break;
//    case USART3_Protocol_Type_Switch:       //221 U3模式改为VISCA
//    {
//        USART3_Protocol_Type = Protocol_VISCA;
//        PT_Position_Record[7] = USART3_Protocol_Type;
//        EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record,Parameter_Bytes_NUN);
//    }
//    break;
    case Self_Adaption_Switch:        //222开启自适应
    {
        Self_Adaption = Self_Adaption_Enable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Self_Adaption;
        EEPRom_WriteDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Home_Position:               //223水平垂直回原点
    {
        Pan_Motion_Mode = Location_Motion;
        Tilt_Motion_Mode = Location_Motion;
        Pan_Target_Angle = Pan_Origin_User;
        Tilt_Target_Angle = Tilt_Origin_User;
        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;     //使用最大速度回原点
        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
        Place_InOut_Pan = OutPlace;
        Place_InOut_Tilt = OutPlace;
    }
    break;
    case Limit_Pan:                   //225打开水平限位
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Enable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Pan_User_Limit_Switch;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Limit_Right:                 //226设置右限位
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_User_Limit_R = Pan_Real_Angle;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Right,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Limit_Left:                  //227设置左限位
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[5] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[6] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[7] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_User_Limit_L = Pan_Real_Angle;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Left,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228设置守望位并开启
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        PT_Position_Record[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        Watcher_Angle_Pan = Pan_Real_Angle;
        Watcher_Angle_Tilt = Tilt_Real_Angle;
        Watcher_Switch = ON;
        watcher_pause_state = watcher_continue; //守望位暂停状态改为‘继续’
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_Set,PT_Position_Record,Parameter_Bytes_NUN);   //记录守望位坐标
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = 0x00;
        PT_Position_Record[3] = 0x00;
        PT_Position_Record[4] = 0x00;
        PT_Position_Record[5] = 0x00;
        PT_Position_Record[6] = 0x00;
        PT_Position_Record[7] = 0x00;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record,Parameter_Bytes_NUN);    //将守望位开关记录
    }
    break;
    case Delete_Data:                 //设置3次230清除所有用户预置位，不包括特殊预置位，不影响守望位
    {
        Delete_Preset_FLag ++;
        if(Delete_Preset_FLag == 3)
        {
            Axial = 0;
            Delete_Preset_FLag = 0;
            USART1_DISABLE();
            TIM_SetCompare1(TIM9,0);
            TIM_SetCompare2(TIM9,0);
            delay_us(10000);
            for(i1 = 1; i1 < (EE_TYPE / Preset_Bytes_NUN); i1 ++)   //eetype对应的是不同型号eeprom的字节数，每个预置位需要存储16个字节
            {
                EEPRom_WriteDataLen(EEPROM_Preset,i1,ERASE_EE_Preset,Preset_Bytes_NUN); //EEPROM_Preset是用户预置位存储的EEPROM
                //*****************************
                //喂狗
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            if(pan_encoder_type == 3 && tilt_encoder_type == 3) //如果编码器是ICMU，则需要记录编码器的当前值
            {
                STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
            }
            delay_us(10000);
            SoftReset();        //软复位
        }
    }
    break;
    case Section_Border1:             //231设置扇区边界1
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        PT_Position_Record[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Section_Border1,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Section_Border2:             //232设置扇区边界2
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        PT_Position_Record[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Section_Border2,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Switch:                 //233启动扇区扫描
    {
        PT_Scan1 = EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border1 - 1) * Parameter_Bytes_NUN);
        PT_Scan2 = EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border2 - 1) * Parameter_Bytes_NUN);
        if(PT_Scan1 == 0xaa && PT_Scan2 == 0xaa)
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border1,Scan_Border1,Parameter_Bytes_NUN);
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border2,Scan_Border2,Parameter_Bytes_NUN);
            if(EEPRom_ReadOneByte(EEPROM_Parameter,(Scan_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)
            {
                EEPRom_ReadDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Extract,Parameter_Bytes_NUN);
                Scan_Speed = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
            }
            else
            {
                        if (Speed_Set_Satus != 0)
				{
					Scan_Speed = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 2;
				}
				else
				{
					Scan_Speed = Scan_Speed_Mid;
				}
            }
            data_p1 = 0;
            data_p1 |= Scan_Border1[2];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1[3];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1[4];
            data_p2 = 0;
            data_p2 |= Scan_Border2[2];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2[3];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2[4];
            Section_Border1_Angle = ((double)data_p1) / 1000.0;
            Section_Border2_Angle = ((double)data_p2) / 1000.0;
            Special_Function = Scan_Enable;
            Pan_Location_Speed = Scan_Speed;
            Pan_Motion_Mode = Location_Motion;
            Place_InOut_Pan = OutPlace;
        }
        else
        {
            Pan_Motion_Mode = Rocker_Motion;
            Pan_Target_Speed = Scan_Speed;
        }
    }
    break;
    case Scan_Switch_1:                 //234启动大于180度扇区扫描
    {
        PT_Scan1 = EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border1 - 1) * Parameter_Bytes_NUN);
        PT_Scan2 = EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border2 - 1) * Parameter_Bytes_NUN);
        if(PT_Scan1 == 0xaa && PT_Scan2 == 0xaa)
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border1,Scan_Border1,Parameter_Bytes_NUN);
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border2,Scan_Border2,Parameter_Bytes_NUN);
            if(EEPRom_ReadOneByte(EEPROM_Parameter,(Scan_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)
            {
                EEPRom_ReadDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Extract,Parameter_Bytes_NUN);
                Scan_Speed = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
            }
            else
            {
                        if (Speed_Set_Satus != 0)
				{
					Scan_Speed = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 2;
				}
				else
				{
					Scan_Speed = Scan_Speed_Mid;
				}
            }
            data_p1 = 0;
            data_p1 |= Scan_Border1[2];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1[3];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1[4];
            data_p2 = 0;
            data_p2 |= Scan_Border2[5];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2[6];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2[7];
            Section_Border1_Angle = ((double)data_p1) / 1000.0;
            Section_Border2_Angle = ((double)data_p2) / 1000.0;
            Special_Function = Scan_Enable;
            Pan_Location_Speed = Scan_Speed;
            Pan_Motion_Mode = Location_Motion_Scan;
            Place_InOut_Pan = OutPlace;
        }
    }
    break;
//        case Normal_Speed_Swtich:
//        {
//            Normal_Speed = Low_Leve;
//        }break;
    case Query_Versions:
        break;              //235查询云台版本
////***********************************************************************************************
////jing210714
//      case Tilt_Inversion:                    //249璋冩暣鍨傜洿瑙掑害澧為暱鏂瑰悜
//          Tilt_Encoder_Direction = Encoder_Forward;
//            memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
//            PT_Position_Record[7] = Tilt_Encoder_Direction;
//            EEPRom_WriteDataLen(EEPROM_Parameter,Tilt_Inversion,PT_Position_Record,Parameter_Bytes_NUN);
//      break;
////***********************************************************************************************
    default:
        break;
    }
}
void Special_Preset_Delete(u16 num)
{
    switch(num)
    {
    case Origin_Pan:                  //201删除水平原点
    {
        Pan_Origin_User = 0;
        EEPRom_WriteOneByte(EEPROM_Parameter,(Origin_Pan - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Origin_Tilt:                 //202删除垂直原点
    {
        Tilt_Origin_User = 0;
        EEPRom_WriteOneByte(EEPROM_Parameter,(Origin_Tilt - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Tour_Speed_Level:            //205巡检速度中等
    {
        if (Speed_Set_Satus == 0)
        {
            Tour_Speed_Pan = Tour_Speed_Max / 2;
            Tour_Speed_Tilt = Tour_Speed_Max / 2;
        }
        else
        {
            Tour_Speed_Pan = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 2;
            Tour_Speed_Tilt = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt / 2;
        }
				Pan_Location_Speed = Tour_Speed_Pan;
				Tilt_Location_Speed = Tour_Speed_Tilt;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[4] = Tour_Speed_Pan >> 8;
        PT_Position_Record[5] = Tour_Speed_Pan & 0xff;
        PT_Position_Record[6] = Tour_Speed_Tilt >> 8;
        PT_Position_Record[7] = Tour_Speed_Tilt & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Level:         //207巡检停留时间中等
    {
        Tour_Time = Tour_WaitTime_Mid;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Level:      //209守望停留时间中等
    {
        Watcher_Time = Watcher_WaitTime_Mid;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Level:            //211扫描速度中等
    {
        Scan_Speed = Scan_Speed_Mid;
        if (Speed_Set_Satus == 0)
        {
            Scan_Speed = Scan_Speed_Mid;
        }
        else
        {
            Scan_Speed = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 2;
        }
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Scan_Speed >> 8;
        PT_Position_Record[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228清除守望位
    {
        memcpy(PT_Position_Record,ERASE_EE_Parameter,Parameter_Bytes_NUN);
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record,Parameter_Bytes_NUN);    //将守望位开关清除
        Watcher_Switch = OFF;
        watcher_pause_state = watcher_pause;    //守望位暂停状态改为‘继续’
    }
    break;
    case Section_Border1:             //231删除扇区边界1
    {
        EEPRom_WriteOneByte(EEPROM_Parameter,(Section_Border1 - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Section_Border2:             //232删除扇区边界2
    {
        EEPRom_WriteOneByte(EEPROM_Parameter,(Section_Border2 - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
//        case 1000:
//        {
//            EEPRom_WriteOneByte(EEPROM_Parameter,(1000 - 1) * Parameter_Bytes_NUN,0xff);
//        }break;
    default:
        break;
    }
}
u32 tttt = 0;
void Special_Preset_Extract(u16 num)
{
    //Q_ERR err_q;
    u16 i3 = 0,i1 = 0,i2 = 0;
    u32 data_p = 0,data_t = 0;
    switch(num)
    {
    case Angle_Return_Mode:           //203查询回传模式
    {
        Return_Mode = Return_Query;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_Speed_Regulation:       //204巡检速度减少
    {
        if (Speed_Set_Satus == 0)
        {
            Tour_Speed_Pan = (Tour_Speed_Pan - Speed_Offset < Pan_Ultimate_MinSpeed * Speed_Scale_Pan) ? Tour_Speed_Min : Tour_Speed_Pan - Speed_Offset;
            Tour_Speed_Tilt = (Tour_Speed_Tilt - Speed_Offset < Tilt_Ultimate_MinSpeed * Speed_Scale_Tilt) ? Tour_Speed_Min : Tour_Speed_Tilt - Speed_Offset;
        }
        else
        {
            Tour_Speed_Pan = (Tour_Speed_Pan - Speed_Offset < Pan_Ultimate_MinSpeed * Speed_Scale_Pan) ? Pan_Ultimate_MinSpeed * Speed_Scale_Pan : Tour_Speed_Pan - Speed_Offset;
            Tour_Speed_Tilt = (Tour_Speed_Tilt - Speed_Offset < Tilt_Ultimate_MinSpeed * Speed_Scale_Tilt) ? Tilt_Ultimate_MinSpeed * Speed_Scale_Tilt : Tour_Speed_Tilt - Speed_Offset;
        }
				Pan_Location_Speed = Tour_Speed_Pan;
				Tilt_Location_Speed = Tour_Speed_Tilt;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[4] = Tour_Speed_Pan >> 8;
        PT_Position_Record[5] = Tour_Speed_Pan & 0xff;
        PT_Position_Record[6] = Tour_Speed_Tilt >> 8;
        PT_Position_Record[7] = Tour_Speed_Tilt & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_Speed_Level:            //205巡检速度最小
    {
        if (Speed_Set_Satus == 0)
        {
            Tour_Speed_Pan = Tour_Speed_Min;
            Tour_Speed_Tilt = Tour_Speed_Min;
        }
        else
        {
            Tour_Speed_Pan =  Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 3;
            Tour_Speed_Tilt = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt / 3;
        }
				Pan_Location_Speed = Tour_Speed_Pan;
				Tilt_Location_Speed = Tour_Speed_Tilt;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[4] = Tour_Speed_Pan >> 8;
        PT_Position_Record[5] = Tour_Speed_Pan & 0xff;
        PT_Position_Record[6] = Tour_Speed_Tilt >> 8;
        PT_Position_Record[7] = Tour_Speed_Tilt & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Regulation:    //206巡检停留时间减少
    {
        if(Tour_Time > Tour_WaitTime_Min)
        {
            Tour_Time = Tour_Time - WaitTime_Offset;
        }
        else
        {
            Tour_Time = Tour_WaitTime_Min;
        }
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Level:         //207巡检停留时间最小
    {
        Tour_Time = Tour_WaitTime_Min;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Regulation: //208守望停留时间减少
    {
        if(Watcher_Time > Watcher_WaitTime_Min)
        {
            Watcher_Time = Watcher_Time - WaitTime_Offset;
        }
        else
        {
            Watcher_Time = Watcher_WaitTime_Min;
        }
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Level:      //209守望停留时间最小
    {
        Watcher_Time = Watcher_WaitTime_Min;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Regulation:       //210扫描速度减少
    {
        if (Speed_Set_Satus == 0)
        {
            Scan_Speed = (Scan_Speed - Speed_Offset < Pan_Ultimate_MinSpeed * Speed_Scale_Pan) ? Scan_Speed_Min : Scan_Speed - Speed_Offset;
        }
        else
        {
            Scan_Speed = (Scan_Speed - Speed_Offset < Pan_Ultimate_MinSpeed * Speed_Scale_Pan) ? Pan_Ultimate_MinSpeed * Speed_Scale_Pan : Scan_Speed - Speed_Offset;
        }
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Scan_Speed >> 8;
        PT_Position_Record[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Level:            //211扫描速度最小
    {
        if (Speed_Set_Satus == 0)
        {
            Scan_Speed = Scan_Speed_Min;
        }
        else
        {
            Scan_Speed =  Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 3;
        }
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Scan_Speed >> 8;
        PT_Position_Record[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Return_Query_485:            //212单485模式
    {
        Return_Mode = Return_Query_485;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Return_Type:                 //213坐标回传
    {
        Return_Data_Type = Return_Position;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Data_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,Return_Type,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
//        case Laser_Adjust_Switch_UNV:       //162退出激光灯调节模式,宇视
    case Laser_Adjust_Switch:         //214退出激光灯调节模式
    {
        Axial = 0;
//            Post_Information_Function(Sem_USART3_TX,&USART3_TX_BOX,Preset_Extract,Laser_Adjust_Switch);
    }
    break;
//    case USART3_Protocol_Type_Switch:       //221 U3模式改为PELCO
//    {
//        USART3_Protocol_Type = Protocol_PELCO_D;
//        PT_Position_Record[7] = USART3_Protocol_Type;
//        EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record,Parameter_Bytes_NUN);
//    }
//    break;
    case Self_Adaption_Switch:        //222关闭自适应
    {
        Self_Adaption = Self_Adaption_Disable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Self_Adaption;
        EEPRom_WriteDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Home_Position:               //223水平180，垂直回原点
    {
        Pan_Motion_Mode = Location_Motion;
        Tilt_Motion_Mode = Location_Motion;
        if(Pan_Origin_User + 180 < 360)
        {
            Pan_Target_Angle = Pan_Origin_User + 180;
        }
        else
        {
            Pan_Target_Angle = Pan_Origin_User - 180;
        }
        Tilt_Target_Angle = Tilt_Origin_User;
        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;     //使用最大速度回原点
        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
        Place_InOut_Pan = OutPlace;
        Place_InOut_Tilt = OutPlace;
    }
    break;
    case Limit_Pan:                   //225关闭水平限位
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Disable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Pan_User_Limit_Switch;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228开启守望位
    {
        if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_Set - 1) * Parameter_Bytes_NUN) == 0xaa)//228用户守望位
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,Watcher_Set,PT_Position_Extract,Parameter_Bytes_NUN);
            data_p = 0;
            data_p |= PT_Position_Extract[2];
            data_p <<= 8;
            data_p |= PT_Position_Extract[3];
            data_p <<= 8;
            data_p |= PT_Position_Extract[4];
            data_t = 0;
            data_t |= PT_Position_Extract[5];
            data_t <<= 8;
            data_t |= PT_Position_Extract[6];
            data_t <<= 8;
            data_t |= PT_Position_Extract[7];
            Watcher_Angle_Pan = ((double)data_p) / 1000.0;
            Watcher_Angle_Tilt = ((double)data_t) / 1000.0;
            memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
            PT_Position_Record[2] = 0x00;
            PT_Position_Record[3] = 0x00;
            PT_Position_Record[4] = 0x00;
            PT_Position_Record[5] = 0x00;
            PT_Position_Record[6] = 0x00;
            PT_Position_Record[7] = 0x00;
            EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record,Parameter_Bytes_NUN);    //将守望位开关记录
            Watcher_Switch = ON;    //开启守望位
            watcher_pause_state = watcher_continue;
        }
        else
        {
            Watcher_Switch = OFF;
            watcher_pause_state = watcher_pause;
        }
    }
    break;
    case Tour_Switch:                 //229开启巡检
    {
        watcher_pause_state = watcher_pause;    //守望位暂停状态改为‘暂停’
        Preset_Number = 0;
        Pan_Target_Speed = 0;
        Stay_Time = 0;
        Pan_Motion_Mode = Rocker_Motion;
        Tilt_Motion_Mode = Rocker_Motion;
        Tilt_Target_Speed = 0;
        if(PWM_P == 0 && PWM_T == 0)
        {
            for(i3 = 1; i3 < 1024; i3 ++)
            {
                //if((i3 < 200 && i3 != 105 && i3 != 111 && i3 != 114 && i3 != 116) || (i3 > 256))
                if( ( i3 < 200 ) || ( i3 > 256 ) )
                {
                    if(0xaa == EEPRom_ReadOneByte(EEPROM_Preset,(i3 - 1) * Preset_Bytes_NUN))
                    {
                        Preset_Num[Preset_Number] = i3;
                        Preset_Number ++;
                    }
                }
            }
            if(Preset_Number > 0)
            {
                Tour_Num = 0;
                if(EEPRom_ReadOneByte(EEPROM_Parameter,(Tour_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)
                {
                    EEPRom_ReadDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Extract,Parameter_Bytes_NUN);
                    Tour_Speed_Pan = ((PT_Position_Extract[4] | 0x0000) << 8) + PT_Position_Extract[5];
                    Tour_Speed_Tilt = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
                }
                else
                {
        if (Speed_Set_Satus != 0)
				{
					Tour_Speed_Pan = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 2;
					Tour_Speed_Tilt = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt / 2;
				}
				else
				{
					Tour_Speed_Pan = Tour_Speed_Mid;
					Tour_Speed_Tilt = Tour_Speed_Mid;
				}
                }
                Pan_Location_Speed = Tour_Speed_Pan;
                Tilt_Location_Speed = Tour_Speed_Tilt;
                if(Preset_Number != 1)
                {
                    Special_Function = Tour_Enable;
                    Stay_Time = Tour_Time;
                    Tilt_Timer_Start = 1;
                    Pan_Timer_Start = 1;
                    Place_InOut_Pan = 1;
                    Place_InOut_Tilt = 1;
                    //Pan_Motion_Mode = Location_Motion;
                    //Tilt_Motion_Mode = Location_Motion;
                }
            }
        }
    }
    break;
    case Delete_Data:                 //调用3次230清除所有数据包括特殊预置位，删除守望位
    {
        Delete_Data_FLag ++;
        if(Delete_Data_FLag == 3)
        {
            Axial = 0;
            Delete_Data_FLag = 0;
            USART1_DISABLE();
            TIM_SetCompare1(TIM9,0);
            TIM_SetCompare2(TIM9,0);
            for(i1 = 1; i1 < (EE_TYPE / sizeof(ERASE_EE_Preset)); i1 ++)
            {
                EEPRom_WriteDataLen(EEPROM_Preset,i1,ERASE_EE_Preset,sizeof(ERASE_EE_Preset));
                //*****************************
                //喂狗
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            for(i2 = 1; i2 < 256; i2 ++)
            {
                EEPRom_WriteDataLen(EEPROM_Parameter,i2,ERASE_EE_Parameter,sizeof(ERASE_EE_Parameter));
                //*****************************
                //喂狗
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            if(pan_encoder_type == 3 && tilt_encoder_type == 3) //如果编码器是ICMU则需要记录编码器当前数值
            {
                STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
            }
            delay_us(10000);
            SoftReset();//直接复位
        }
    }
    break;
    case Scan_Switch:                 //233停止特殊功能（巡检，扫描）
    {
        Special_Function = Tour_Scan_Disable;
    }
    break;
    case Query_Versions:
        break;        //235查询镜头版本
////***********************************************************************************************
////jing210714
//      case Tilt_Inversion:                    //249璋冩暣鍨傜洿瑙掑害澧為暱鏂瑰悜
//          Tilt_Encoder_Direction = Encoder_Inversion;
//            memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
//            PT_Position_Record[7] = Tilt_Encoder_Direction;
//            EEPRom_WriteDataLen(EEPROM_Parameter,Tilt_Inversion,PT_Position_Record,Parameter_Bytes_NUN);
//      break;
////***********************************************************************************************
    default:
        break;
    }
}
//*************************************************************************
u8 Origin_Machine_Data[8];
u32 Parameters_Init(void)
{
    volatile u8 send_data[30] = {0};
    u32 pwr_data_encoder[4] = {0};
    u32 flash_data_crc = 0;
    u8 state;
    //u8 i;
    u32 data_p = 0,data_t = 0,data_p1 = 0,data_p2 = 0;
    state = EEPRom_ReadOneByte(EEPROM_Parameter,(Machine_Origin - 1) * Parameter_Bytes_NUN);
    //以下在上电后读取FLASH在上次掉电时保存的数据
    pwr_data_encoder[0] = STMFLASH_ReadWord(track_offect);      //从FLASH中读取水平编码器数据
    pwr_data_encoder[1] = STMFLASH_ReadWord(track_offect + 4);  //从FLASH中读取俯仰编码器数据
    pwr_data_encoder[2] = STMFLASH_ReadWord(track_offect + 8);  //从FLASH中读取上电后读取编码器次数，该次数会溢出
    pwr_data_encoder[3] = STMFLASH_ReadWord(track_offect + 12); //从FLASH中读取CRC校验值
    STM32Sector_Erase(track_offect);    //读取完毕掉电保存的水平和垂直数据后对该区域进行擦除
    if(CRC_16((u8*)pwr_data_encoder,12) != (u16)pwr_data_encoder[3])    //判断校验值是否正确
    {
        flash_data_crc = 1; //flash_data_crc为1表示CRC错误
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Machine_Origin - 1) * Parameter_Bytes_NUN) == 0xaa)
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Machine_Origin,PT_Position_Extract,Parameter_Bytes_NUN);
        switch(pan_encoder_type)
        {
        case 1:
        {
            Pan_Machine_Origin_Angle = (double)(((PT_Position_Extract[4] | 0x0000) << 8) + PT_Position_Extract[5]) / 100.0;
        }
        break;
        case 2:
        {
            Pan_Machine_Origin = ((PT_Position_Extract[4] | 0x0000) << 8) + PT_Position_Extract[5];
        }
        break;
        case 3:
        {
            Pan_Machine_Origin = (((PT_Position_Extract[2] | 0x00000000) << 16)) + ((PT_Position_Extract[3] | 0x00000000) << 8) + PT_Position_Extract[4];
            if(!flash_data_crc)     //如果掉电FLASH中保存的数据可靠，即CRC验证正确
            {
                pan_track_err = (int)(((pwr_data_encoder[0] & 0x3ffff) - Pan_Encoder_Data_Last) << 14);
                pan_track_err /= 16384;
                pan_track_offset = pan_track_err / 8192 + (pan_track_err % 8192) / 4096;
                send_data[0] = 0xaa;
                send_data[4] = 0xaa;
                send_data[8] = 0xaa;
            }
            else            //如果掉电保存的数据不可靠
            {
                send_data[0] = 0xee;
                send_data[4] = 0xee;
                send_data[8] = 0xee;
            }
            send_data[1] = pwr_data_encoder[0] >> 16;
            send_data[2] = pwr_data_encoder[0] >> 8;
            send_data[3] = pwr_data_encoder[0];
            send_data[5] = Pan_Encoder_Data_Last >> 16;
            send_data[6] = Pan_Encoder_Data_Last >> 8;
            send_data[7] = Pan_Encoder_Data_Last;
            send_data[9] = pan_track_offset;
        }
        break;
        case 4:
        {
            Pan_Machine_Origin = (((PT_Position_Extract[2] | 0x00000000) << 16)) + ((PT_Position_Extract[3] | 0x00000000) << 8) + PT_Position_Extract[4];
        }
        break;
        default:
        {
            Pan_Machine_Origin = 0;
        }
        break;
        }
        switch(tilt_encoder_type)
        {
        case 1:
        {
            Tilt_Machine_Origin_Angle = (double)(((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7]) / 100.0;
        }
        break;
        case 2:
        {
            Tilt_Machine_Origin = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
            Origin_Machine_Flag = 5;
        }
        break;
        case 3:
        {
            Tilt_Machine_Origin = ((PT_Position_Extract[5] | 0x00000000) << 16) + ((PT_Position_Extract[6] | 0x00000000) << 8) + PT_Position_Extract[7];
            Origin_Machine_Flag = 5;
            if(!flash_data_crc)
            {
//                    #if defined (PT_EXF)
//                    tilt_track_err = (int)((Tilt_Encoder_Data_Last - (pwr_data_encoder[1] & 0x3ffff)) << 14);
//                    #else
                tilt_track_err = (int)(((pwr_data_encoder[1] & 0x3ffff) - Tilt_Encoder_Data_Last) << 14);
//                    #endif
                tilt_track_err /= 16384;
                tilt_track_offset = tilt_track_err / 8192 + (tilt_track_err % 8192) / 4096;
                send_data[10] = 0xaa;
                send_data[14] = 0xaa;
                send_data[18] = 0xaa;
                send_data[20] = 0xaa;
                send_data[24] = 0xaa;
            }
            else
            {
                send_data[10] = 0xee;
                send_data[14] = 0xee;
                send_data[18] = 0xee;
                send_data[20] = 0xee;
                send_data[24] = 0xee;
            }
            send_data[11] = pwr_data_encoder[1] >> 16;
            send_data[12] = pwr_data_encoder[1] >> 8;
            send_data[13] = pwr_data_encoder[1];
            send_data[15] = Tilt_Encoder_Data_Last >> 16;
            send_data[16] = Tilt_Encoder_Data_Last >> 8;
            send_data[17] = Tilt_Encoder_Data_Last;
            send_data[19] = tilt_track_offset;
            send_data[21] = pwr_data_encoder[2] >> 16;
            send_data[22] = pwr_data_encoder[2] >> 8;
            send_data[23] = pwr_data_encoder[2];
            send_data[25] = flash_data_crc;
        }
        break;
        case 4:
        {
            Tilt_Machine_Origin = ((PT_Position_Extract[5] | 0x00000000) << 16) + ((PT_Position_Extract[6] | 0x00000000) << 8) + PT_Position_Extract[7];
            Origin_Machine_Flag = 5;
        }
        break;
        default:
        {
            Tilt_Machine_Origin = 0;
        }
        break;
        }
    }
    else
    {
        Origin_Machine_Flag = 0;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Machine_Up_EE - 1) * Parameter_Bytes_NUN) == 0xaa)//0x01ff
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Limit_Machine_Up_EE,PT_Position_Extract,Parameter_Bytes_NUN);
        Tilt_Limit_Up = (double)(((u16)PT_Position_Extract[6] << 8) + PT_Position_Extract[7]) / 100;
    }
    else
    {
        Tilt_Limit_Up = Tilt_Angle_Limit_Up_default;    //如果未设置限位角度，则按照默认限位角度进行设置
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Machine_Down_EE - 1) * Parameter_Bytes_NUN) == 0xaa)//0x02ff
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Limit_Machine_Down_EE,PT_Position_Extract,Parameter_Bytes_NUN);
        Tilt_Limit_Down = (double)(((u16)PT_Position_Extract[6] << 8) + PT_Position_Extract[7]) / 100;
    }
    else
    {
        Tilt_Limit_Down = Tilt_Angle_Limit_Down_default;    //如果未设置限位角度则按照默认限位角度进行设置
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Tilt_Angle_Direction_EE - 1) * Parameter_Bytes_NUN) == 0xaa)//0x03ff//读出反转标志
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Tilt_Angle_Direction_EE,PT_Position_Extract,Parameter_Bytes_NUN);
        Tilt_Encoder_Direction = PT_Position_Extract[7];
    }
    else
    {
        Tilt_Encoder_Direction = Encoder_Forward;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Pan - 1) * Parameter_Bytes_NUN) == 0xaa)  //200水平限位开关
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Extract,Parameter_Bytes_NUN);
        Pan_User_Limit_Switch = PT_Position_Extract[7];
    }
    else
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Disable;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Origin_Pan - 1) * Parameter_Bytes_NUN) == 0xaa) //201水平原点
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Origin_Pan,PT_Position_Extract,Parameter_Bytes_NUN);
        data_p = 0;
        data_p |= PT_Position_Extract[2];
        data_p <<= 8;
        data_p |= PT_Position_Extract[3];
        data_p <<= 8;
        data_p |= PT_Position_Extract[4];
        Pan_Origin_User = ((double)data_p) / 1000.0;
    }
    else
    {
        Pan_Origin_User = 0;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Origin_Tilt - 1) * Parameter_Bytes_NUN) == 0xaa)    //202垂直原点
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Origin_Tilt,PT_Position_Extract,Parameter_Bytes_NUN);
        data_t = 0;
        data_t |= PT_Position_Extract[5];
        data_t <<= 8;
        data_t |= PT_Position_Extract[6];
        data_t <<= 8;
        data_t |= PT_Position_Extract[7];
        Tilt_Origin_User = ((double)data_t) / 1000.0;
    }
    else
    {
        Tilt_Origin_User = 0;
    }
		    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Max_Speed - 1) * Preset_Bytes_NUN) == 0xaa)//最高速度
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Max_Speed,PT_Position_Extract,Preset_Bytes_NUN);
        if(PT_Position_Extract[1] == 0x00)
        {
            Pan_Ultimate_MaxSpeed = 4000;
            Tilt_Ultimate_MaxSpeed = 1000;
            Pan_Ultimate_MinSpeed = A_MinSpeed;
            Tilt_Ultimate_MinSpeed = A_MinSpeed;
            Scale_Velocity_Pan = 4.0;
            Scale_Velocity_Tilt = 1.1;
            Speed_Scale_Pan = 1.0;
            Speed_Scale_Tilt = 1.0;
            Speed_Set_Satus = 1;
        }
        else
        {
            Pan_Ultimate_MaxSpeed = ((PT_Position_Extract[1] | 0x0000) << 8) + PT_Position_Extract[2];
            Tilt_Ultimate_MaxSpeed = ((PT_Position_Extract[3] | 0x0000) << 8) + PT_Position_Extract[4];
            Pan_Ultimate_MinSpeed = ((PT_Position_Extract[5] | 0x0000) << 8) + PT_Position_Extract[6];
            Tilt_Ultimate_MinSpeed = ((PT_Position_Extract[7] | 0x0000) << 8) + PT_Position_Extract[8];
            Scale_Velocity_Pan = (float)(((PT_Position_Extract[9] | 0x0000) << 8) + PT_Position_Extract[10]) / 100.0f;
            Scale_Velocity_Tilt = (float)(((PT_Position_Extract[11] | 0x0000) << 8) + PT_Position_Extract[12]) / 100.0f ;
            Speed_Scale_Pan = (float)PT_Position_Extract[13] / 100.0f;
            Speed_Scale_Tilt = (float)PT_Position_Extract[14] / 100.0f;
            Speed_Set_Satus = PT_Position_Extract[15];
        }
    }
    else
    {
        Pan_Ultimate_MaxSpeed = 4000;
        Tilt_Ultimate_MaxSpeed = 1000;
        Pan_Ultimate_MinSpeed = A_MinSpeed;
        Tilt_Ultimate_MinSpeed = A_MinSpeed;
        Scale_Velocity_Pan = 4.0;
        Scale_Velocity_Tilt = 1.1;
        Speed_Scale_Pan = 1.0;
        Speed_Scale_Tilt = 1.0;
        Speed_Set_Satus = 1;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Angle_Return_Mode - 1) * Parameter_Bytes_NUN) == 0xaa)//203回传方式，查询or实时
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Extract,8);
        Return_Mode = PT_Position_Extract[7];
    }
    else
    {
        Return_Mode = Return_Query_485;
        //Return_Mode = Return_Query;
//        Return_Mode = Return_RealTime;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Tour_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//205巡检速度
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Extract,Parameter_Bytes_NUN);
        Tour_Speed_Pan = ((PT_Position_Extract[4] | 0x0000) << 8) + PT_Position_Extract[5];
        Tour_Speed_Tilt = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
    }
    else
    {
        if (Speed_Set_Satus != 0)
				{
					Tour_Speed_Pan = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 2;
					Tour_Speed_Tilt = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt / 2;
				}
				else
				{
					Tour_Speed_Pan = Tour_Speed_Mid;
					Tour_Speed_Tilt = Tour_Speed_Mid;
				}
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Tour_StayTime_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//207巡检停留时间
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Extract,Parameter_Bytes_NUN);
        Tour_Time = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
    }
    else
    {
        Tour_Time = Tour_WaitTime_Min;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_StayTime_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//209回守望位等待时间
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Extract,Parameter_Bytes_NUN);
        Watcher_Time = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
    }
    else
    {
        Watcher_Time = Watcher_WaitTime_Mid;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Scan_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//211扫描速度
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Extract,Parameter_Bytes_NUN);
        Scan_Speed = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
    }
    else
    {
        if (Speed_Set_Satus != 0)
				{
					Scan_Speed = Pan_Ultimate_MaxSpeed * Speed_Scale_Pan / 2;
				}
				else
				{
					Scan_Speed = Scan_Speed_Mid;
				}
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Angle_Return_Mode - 1) * Parameter_Bytes_NUN) == 0xaa)  //212单485模式
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Extract,Parameter_Bytes_NUN);
        Return_Mode = PT_Position_Extract[7];
    }
    else
    {
        Return_Mode = Return_Query_485;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record,Parameter_Bytes_NUN);
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Return_Type - 1) * Parameter_Bytes_NUN) == 0xaa)//213回传数据类型，角度or编码器值
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Return_Type,PT_Position_Extract,8);
        Return_Data_Type = PT_Position_Extract[7];
    }
    else
    {
        Return_Data_Type = Return_Angle;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Self_Adaption_Switch - 1) * Parameter_Bytes_NUN) == 0xaa)//222自适应开关
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Extract,Parameter_Bytes_NUN);
        Self_Adaption = PT_Position_Extract[7];
    }
    else
    {
        Self_Adaption = Self_Adaption_Disable;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Right - 1) * Parameter_Bytes_NUN) == 0xaa)//226用户右限位
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Limit_Right,PT_Position_Extract,Parameter_Bytes_NUN);
        data_p1 = 0;
        data_p1 |= PT_Position_Extract[2];
        data_p1 <<= 8;
        data_p1 |= PT_Position_Extract[3];
        data_p1 <<= 8;
        data_p1 |= PT_Position_Extract[4];
        Pan_User_Limit_R = ((double)data_p1) / 1000.0;
    }
    else
    {
        Pan_User_Limit_R = 0xffff;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Left - 1) * Parameter_Bytes_NUN) == 0xaa)//227用户左限位
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Limit_Left,PT_Position_Extract,Parameter_Bytes_NUN);
        data_p2 = 0;
        data_p2 |= PT_Position_Extract[5];
        data_p2 <<= 8;
        data_p2 |= PT_Position_Extract[6];
        data_p2 <<= 8;
        data_p2 |= PT_Position_Extract[7];
        Pan_User_Limit_L = ((double)data_p2) / 1000.0;
    }
    else
    {
        Pan_User_Limit_L = 0xffff;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_Set - 1) * Parameter_Bytes_NUN) == 0xaa)//228用户守望位
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Watcher_Set,PT_Position_Extract,Parameter_Bytes_NUN);
        data_p = 0;
        data_p |= PT_Position_Extract[2];
        data_p <<= 8;
        data_p |= PT_Position_Extract[3];
        data_p <<= 8;
        data_p |= PT_Position_Extract[4];
        data_t = 0;
        data_t |= PT_Position_Extract[5];
        data_t <<= 8;
        data_t |= PT_Position_Extract[6];
        data_t <<= 8;
        data_t |= PT_Position_Extract[7];
        Watcher_Angle_Pan = ((double)data_p) / 1000.0;
        Watcher_Angle_Tilt = ((double)data_t) / 1000.0;
        //Watcher_Switch = ON;
    }
    //else
    //{
    //    Watcher_Switch = OFF;
    //}
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_ON_OFF - 1) * Parameter_Bytes_NUN) == 0xaa)//228用户守望位
    {
        Watcher_Switch = ON;
    }
    else
    {
        Watcher_Switch = OFF;
    }
//    if(EEPRom_ReadOneByte(EEPROM_Parameter,(USART3_Protocol_Type_Switch - 1) * Parameter_Bytes_NUN) == 0xaa)//U3协议类型
//    {
//        EEPRom_ReadDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Extract,Parameter_Bytes_NUN);
//        USART3_Protocol_Type = PT_Position_Extract[7];
//    }
//    else
//    {
//        USART3_Protocol_Type = Protocol_VISCA;
//    }
    watcher_pause_state = watcher_continue; //将守望位暂停状态设置为继续，如果守望位启动则上电后守望位处于激活状态
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(PID_Manual - 1) * Parameter_Bytes_NUN) == 0xaa)//PID 参数
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,PID_Manual,PT_Position_Extract,Parameter_Bytes_NUN);
        Tilt_Kd =PT_Position_Record[2];
        Tilt_Ki =PT_Position_Record[3];
        Tilt_Kp =PT_Position_Record[4];
        Pan_Kd =PT_Position_Record[5];
        Pan_Ki  =PT_Position_Record[6];
        Pan_Kp =PT_Position_Record[7];
        Pan_Kp /= 100;
        Pan_Ki /= 100;
        Pan_Kd /= 100;
        Tilt_Kp /= 1000;
        Tilt_Ki /= 1000;
        Tilt_Kd /= 1000;
        if (!Tilt_Kd || !Tilt_Ki|| !Tilt_Kp)
        {
            Tilt_Kp = T_Kp;
            Tilt_Ki = T_Ki;
            Tilt_Kd = T_Kd;
        }
        if (!Pan_Kd || !Pan_Ki|| !Pan_Kp)
        {
            Pan_Kp = P_Kp;
            Pan_Ki = P_Ki;
            Pan_Kd = P_Kd;
        }
    }
    else
    {
        Pan_Kp = P_Kp;
        Pan_Ki = P_Ki;
        Pan_Kd = P_Kd;
        Tilt_Kp = T_Kp;
        Tilt_Ki = T_Ki;
        Tilt_Kd = T_Kd;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Filter_EE - 1) * Parameter_Bytes_NUN) == 0xaa)//滤波器参数
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Filter_EE,PT_Position_Extract,Parameter_Bytes_NUN);
        pan_data.Main_Switch = PT_Position_Record[4];
        tilt_data.Main_Switch = PT_Position_Record[5];
        pan_data.length = PT_Position_Record[6];
        tilt_data.length = PT_Position_Record[7];
    }
    else
    {
        pan_data.Main_Switch = 0;
        tilt_data.Main_Switch = 0;
        pan_data.length = 10;
        tilt_data.length = 10;//??????鳤??
    }
    if (EEPRom_ReadOneByte(EEPROM_Parameter,(103 - 1) * Parameter_Bytes_NUN) == 0xaa)   //水平上电角度
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,103,PT_Position_Extract,Parameter_Bytes_NUN);
        data_p = 0;
        data_p |= PT_Position_Extract[2];
        data_p <<= 8;
        data_p |= PT_Position_Extract[3];
        data_p <<= 8;
        data_p |= PT_Position_Extract[4];
        Pan_Angle_PowerBoot = ((double)data_p) / 1000.0;
        Pan_Motion_Mode = Location_Motion;
        Pan_Target_Angle = Pan_Angle_PowerBoot;
        Place_InOut_Pan = OutPlace
    }
    else
    {
        Pan_Angle_PowerBoot = 180.0;
        Pan_Motion_Mode = Location_Motion;
        Pan_Target_Angle = Pan_Angle_PowerBoot;
        Place_InOut_Pan = OutPlace
    }
    if (EEPRom_ReadOneByte(EEPROM_Parameter,(104 - 1) * Parameter_Bytes_NUN) == 0xaa)
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,104,PT_Position_Extract,Parameter_Bytes_NUN);
        data_t = 0;
        data_t |= PT_Position_Extract[2];
        data_t <<= 8;
        data_t |= PT_Position_Extract[3];
        data_t <<= 8;
        data_t |= PT_Position_Extract[4];
        Tilt_Angle_PowerBoot = ((double)data_t) / 1000.0;
        Tilt_Motion_Mode = Location_Motion;
        Tilt_Target_Angle = Tilt_Angle_PowerBoot;
        Place_InOut_Tilt = OutPlace
    }
    else
    {
        Tilt_Angle_PowerBoot = 180.0;
        Tilt_Motion_Mode = Location_Motion;
        Tilt_Target_Angle = Tilt_Angle_PowerBoot;
        Place_InOut_Tilt = OutPlace
    }
    return state;
}
//***********************************************
//等比数列
#define a1  28.0
#define q   1.09
//***********************************************
u16 Speed_Level_Pan(u16 speed,u8 state_Self_Adaption)
{
    u16 sp = 0;
    u16 level = 0;
    u16 level_max = 0;
//    u16 i;
//    double p_q = 1.0;
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
//            for(i = 1;i < (speed >> 8);i ++)
//            {
//                p_q *= q;
//            }
//            sp = a1 * p_q;
        sp = (((Pan_Ultimate_MaxSpeed * Speed_Scale_Pan - (Pan_Ultimate_MinSpeed)) * (level)) + (Pan_Ultimate_MinSpeed * Speed_Scale_Pan) * 64 - Pan_Ultimate_MaxSpeed) / 63;
//        sp = (Pan_Ultimate_MaxSpeed*Speed_Scale_Pan /63)*level;
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
//    u16 i = 0;
//    double t_q = 1.0;
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
//            for(i = 1;i < (speed & 0xff);i ++)
//            {
//                t_q *= q;
//            }
//            sp = a1 * t_q;
        sp = (((Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt - (Tilt_Ultimate_MinSpeed)) * (level)) + (Tilt_Ultimate_MinSpeed) * 64 - Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt) / 63;
        //sp = (Tilt_Ultimate_MaxSpeed*Speed_Scale /63)*level;
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
