#include "TaskUSART2.h"
#include "hardware.h"
#include "TaskUSART_TX.h"
#include "TaskAnalyze.h"
#include "TaskUSART3.h"
#include "TaskUSART_TX.h"
#include "24LCxx.h"
#include "hardware.h"
#include "stmflash.h"
#include "TaskEncoder.h"
#include "laser.h"
#include "CRC16.h"
double Pan_Angle_U2 = 0;
double Tilt_Angle_U2 = 0;
u8 PT_Position_Delete_U2[1] = {0xff};
u8 Position_Data_Format_U2[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 PT_Position_Record_U2[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 PT_Position_Extract_U2[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 ERASE_EE_Preset_U2[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
u8 ERASE_EE_Parameter_U2[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
void Task_USART2_RX(void *p)
{
    OS_ERR err;
    p = p;
    USART2_INT_Init(USART2_BAND);
    OSSemCreate ((OS_SEM*   )&Sem_USART2_RX,
                 (CPU_CHAR* )"USART2_RX",
                 (OS_SEM_CTR)0,
                 (OS_ERR*   )&err);
    while(1)
    {
        OSSemPend(&Sem_USART2_RX,0,OS_OPT_PEND_BLOCKING,0,&err);
        //Pend_Information_Function(&USART2_RX_BOX,USART2_RX_Function);
        if(U2_switch)
        {
            USART2_RX_Function();
        }
    }
}
u16 USART2_byte_num = 0;
u8 USART2_byte_buf[300];
void USART2_RX_Function(void)
{
    //数据保存
    for(; USART2_data_pointer != USART2_dispose_data_pointer;)
    {
        USART2_dispose_data_pointer = USART2_dispose_data_pointer % DMA_USART2_RX_BUF_SIZE;
        if(USART2_byte_num == 0)
        {
            if(USART2_RX_Buf[USART2_dispose_data_pointer] == 0xff)
            {
                USART2_byte_buf[USART2_byte_num] = USART2_RX_Buf[USART2_dispose_data_pointer];
                USART2_byte_num ++;
            }
        }
        else
        {
            USART2_byte_buf[USART2_byte_num] = USART2_RX_Buf[USART2_dispose_data_pointer];
            USART2_byte_num ++;
            if (USART2_byte_buf[1] > 0x00 && USART2_byte_buf[1] < 0x0f)
            {
                if(USART2_byte_num > 6)
                {
                    if(USART2_byte_buf[6] == ((USART2_byte_buf[1] + USART2_byte_buf[2] + USART2_byte_buf[3] + USART2_byte_buf[4] + USART2_byte_buf[5]) & 0xff))
                    {
                        if (Analyze_BOX2.Statistic_number<Memory_Finger_NUM)
                        {
                            //填充结构体
                            Analyze_BOX2.Memory_command[Analyze_BOX2.memory_finger]=(USART2_byte_buf[2] << 8) + USART2_byte_buf[3];
                            Analyze_BOX2.Memory_Data[Analyze_BOX2.memory_finger]=(USART2_byte_buf[4] << 8) + USART2_byte_buf[5];
                            Analyze_BOX2.Statistic_number ++;
                            Analyze_BOX2. memory_finger ++;
                            if(Analyze_BOX2.memory_finger >= Memory_Finger_NUM)
                            {
                                Analyze_BOX2.memory_finger = 0;
                            }
                            Pend_Information_Function(&Analyze_BOX2,USART2_RX_Analyze);
                        }
                    }
                    USART2_byte_num = 0;
                }
            }
            else
            {
                USART2_byte_num = 0;
                return;
            }
            //数据处理
        }
        USART2_dispose_data_pointer ++;
        USART2_dispose_data_pointer = USART2_dispose_data_pointer % DMA_USART2_RX_BUF_SIZE;
    }
}
void    USART2_RX_Analyze(void)
{
    {
        //Q_ERR err_q;
        u16 Down_CMD = 0;
        u32 Down_Data = 0;
        u16 i = 0,j = 0;
        u32 data_p = 0,data_t = 0;
        u8 send_len = 0;
        u8 send_data[20] = {0};
        u8 USART_x = 3;
        Down_CMD = Analyze_BOX2.Memory_command[Analyze_BOX2.extract_finger];
        Down_Data = Analyze_BOX2.Memory_Data[Analyze_BOX2.extract_finger];
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
            //****************************************************************
            //????????
        case Direction_Stop:
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
            Pan_Target_Speed = 0;
            Tilt_Target_Speed = 0;
            watcher_pause_state = watcher_continue; //????λ????????????????
            Special_Function = Tour_Scan_Disable;   //?????
            Stay_Time = 0;
            if(USART3_Protocol_Type == Protocol_VISCA)
            {
                memcpy(send_data,CAM_Zoom_Stop,6);
                send_len = 6;
            }
            USART_x = 3;//????3
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
                Special_Function = Tour_Scan_Disable;
                Stay_Time = 0;
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
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
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
            }
            break;
        case Direction_Left:
            if(!Axial)
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
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
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
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
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
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
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
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
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
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
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
                USART_x = 0;//??????
            }
            else
            {
                USART_x = 3;//????3
            }
            break;
            //????????
        case AUX_Switch_ON:
        {
            USART_x = 3;//????3
        }
        break;
        case AUX_Switch_OFF:
        {
            USART_x = 3;//????3
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
        case Laser_ONOFF://             0x0101
            memcpy(send_data,laser_onoff_cmd,send_len);
            send_data[4] = Down_Data >> 8;
            send_data[6] = Check_Sum(send_data,1,6);
            USART_x = 2;
            break;
        case Laser_Current_Change://    0x0102
            memcpy(send_data,laser_current_cmd,send_len);
            send_data[4] = Down_Data >> 8;
            send_data[6] = Check_Sum(send_data,1,6);
            USART_x = 2;
            break;
        case Laser_Current_Set://       0x0103
            memcpy(send_data,laser_current_set,send_len);
            send_data[4] = Down_Data >> 8;
            send_data[6] = Check_Sum(send_data,1,6);
            USART_x = 2;
            break;
        case Laser_Step_Change://       0x0104
            memcpy(send_data,laser_step_decrease,send_len);
            send_data[4] = Down_Data >> 8;
            send_data[5] = Down_Data & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            USART_x = 2;
            break;
        case Laser_Position_Set://      0x0105
            memcpy(send_data,laser_position_set,send_len);
            send_data[4] = Down_Data >> 8;
            send_data[5] = Down_Data & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            USART_x = 2;
            break;
        case Laser_Reset://             0x0106
            memcpy(send_data,laser_reset,send_len);
            send_data[6] = Check_Sum(send_data,1,6);
            USART_x = 2;
            break;
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
            //****************************************************************
            //???λ????
        case Preset_Set:
            if(Down_Data >= 1 && Tilt_Hard_Limit_FLAG == Tilt_Limit_None)
            {
                //if((Down_Data < 200 && Down_Data != 105 && Down_Data != 111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))    //105,111,114,116?????????????λ????????????????
                if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //??????????????λ??????????????????л???????????????
                {
                    memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Preset_Bytes_NUN);
                    PT_Position_Record_U2[8] = Zoom_Position >> 8;
                    PT_Position_Record_U2[9] = Zoom_Position & 0xff;
                    PT_Position_Record_U2[10] = (u32)(Pan_Real_Angle * 1000) >> 16;
                    PT_Position_Record_U2[11] = (u32)(Pan_Real_Angle * 1000) >> 8;
                    PT_Position_Record_U2[12] = (u32)(Pan_Real_Angle * 1000);
                    PT_Position_Record_U2[13] = (u32)(Tilt_Real_Angle * 1000) >> 16;
                    PT_Position_Record_U2[14] = (u32)(Tilt_Real_Angle * 1000) >> 8;
                    PT_Position_Record_U2[15] = (u32)(Tilt_Real_Angle * 1000);
                    EEPRom_WriteDataLen(EEPROM_Preset,Down_Data,PT_Position_Record_U2,Preset_Bytes_NUN);
                }
                else
                {
                    USART2_Special_Preset_Set(Down_Data);
                }
            }
            USART_x = 3;
            break;
        case Preset_Delete:
            //if((Down_Data < 200 && Down_Data != 105 && Down_Data != 111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))
            if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //??????????????λ??????????????????л???????????????
            {
                EEPRom_WriteOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN,0xff);
            }
            else
            {
                USART2_Special_Preset_Delete(Down_Data);
            }
            USART_x = 3;
            break;
        case Preset_Extract:
            //if((Down_Data < 200 && Down_Data != 105 && Down_Data != 111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))
            if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //??????????????λ??????????????????л???????????????
            {
                Stay_Time = 0;
                Special_Function = Tour_Scan_Disable;
                if(EEPRom_ReadOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN) == 0xaa)
                {
                    Pan_Motion_Mode = Location_Motion;
                    Tilt_Motion_Mode = Location_Motion;
                    EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,PT_Position_Extract_U2,Preset_Bytes_NUN);
                    Preset_Empty = 0;
                    CAM_Zoom_Direct[4] = PT_Position_Extract_U2[8] >> 4;
                    CAM_Zoom_Direct[5] = PT_Position_Extract_U2[8] & 0xf;
                    CAM_Zoom_Direct[6] = PT_Position_Extract_U2[9] >> 4;
                    CAM_Zoom_Direct[7] = PT_Position_Extract_U2[9] & 0xf;
                    Zoom_Position = ((u16)PT_Position_Extract_U2[8] << 8) + PT_Position_Extract_U2[9];
                    data_p = 0;
                    data_p |= PT_Position_Extract_U2[10];
                    data_p <<= 8;
                    data_p |= PT_Position_Extract_U2[11];
                    data_p <<= 8;
                    data_p |= PT_Position_Extract_U2[12];
                    data_t = 0;
                    data_t |= PT_Position_Extract_U2[13];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract_U2[14];
                    data_t <<= 8;
                    data_t |= PT_Position_Extract_U2[15];
                    Pan_Target_Angle = ((double)data_p) / 1000.0;
                    Tilt_Target_Angle = ((double)data_t) / 1000.0;
                    if(Special_Function == Tour_Scan_Disable)
                    {
                        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;
                        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
                    }
                    else
                    {
                        Pan_Location_Speed = Tour_Speed;
                        Tilt_Location_Speed = Tour_Speed * 0.6;
                    }
                    Place_InOut_Pan = OutPlace;
                    Place_InOut_Tilt = OutPlace;
//                  Preset_Extract_Flag = 1;
//                  Preset_Extract_num = Down_Data;
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
                Pan_Target_Angle = ((double)Down_Data) / Angle_Decimal + Pan_Origin_User;//??????
            }
            if(Pan_Target_Angle > Circle_Value)
            {
                Pan_Target_Angle = Pan_Target_Angle - Circle_Value;//??????
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
                if(Tilt_Encoder_Direction == Encoder_Inversion)     //????????
                {
                    TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                    if(TTA < 0)
                    {
                        TTA = Circle_Value + TTA;//??????
                    }
                }
                else
                {
                    TTA = ((double)Down_Data) / Angle_Decimal + Tilt_Origin_User;//??????
                    //TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                    if(TTA > Circle_Value)
                    {
                        TTA = TTA - Circle_Value;//??????
                    }
                }
            }
            if(!(TTA > Tilt_Limit_Up && TTA < Tilt_Limit_Down))
            {
                Stay_Time = 0;
                Tilt_Target_Angle = TTA;        //????????????????????
                Special_Function = Tour_Scan_Disable;
                Tilt_Motion_Mode = Location_Motion;
                Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
                Place_InOut_Tilt = OutPlace;
            }
            USART_x = 0;
            break;
            //****************************************************************
            //???????
        case Query_Angle_Pan:
            if (Return_Mode == Return_Query_485)
            {
                Pan_Angle_U2 = Pan_Current_Angle_User;
            }
            else
            {
                Pan_Angle_U2 = pan_angle_query;
            }
            send_data[2] = 0x00;
            send_data[3] = 0x59;
            if(Return_Data_Type == Return_Position)//??????????
            {
                if(pan_encoder_type == Encoder_AS5600)
                {
                    send_data[4] = ((u16)(Pan_Angle_U2 * Pan_Encoder_Range / Circle_Value)) >> 8;
                    send_data[5] = ((u16)(Pan_Angle_U2 * Pan_Encoder_Range / Circle_Value)) & 0xff;
                    send_data[6] = Check_Sum(send_data,1,6);
                    send_len = 7;
                }
                else
                {
                    send_data[4] = ((u32)(Pan_Angle_U2 * Pan_Encoder_Range / Circle_Value)) >> 16;
                    send_data[5] = ((u32)(Pan_Angle_U2 * Pan_Encoder_Range / Circle_Value)) >> 8;
                    send_data[6] = ((u32)(Pan_Angle_U2 * Pan_Encoder_Range / Circle_Value)) & 0xff;
                    send_data[7] = Check_Sum(send_data,1,7);
                    send_len = 8;
                }
            }
            else//?????
            {
                if(Angle_Accuracy_Flag)//?????Э??
                {
                    send_data[4] = ((u32)(Pan_Angle_U2 * 1000)) >> 16;
                    send_data[5] = ((u32)(Pan_Angle_U2 * 1000)) >> 8;
                    send_data[6] = ((u32)(Pan_Angle_U2 * 1000)) & 0xff;
                    send_data[7] = Check_Sum(send_data,1,7);
                    send_len = 8;
                }
                else
                {
                    send_data[4] = ((u16)(Pan_Angle_U2 * 100)) >> 8;
                    send_data[5] = ((u16)(Pan_Angle_U2 * 100)) & 0xff;
                    send_data[6] = Check_Sum(send_data,1,6);
                    send_len = 7;
                }
//                #endif
            }
            USART_x = 2;
            break;
        case Query_Angle_Tilt:
            if (Return_Mode == Return_Query_485)
            {
                Tilt_Angle_U2 = Tilt_Current_Angle_User;
            }
            else
            {
                Tilt_Angle_U2 = tilt_angle_query;
            }
            send_data[2] = 0x00;
            send_data[3] = 0x5b;
            if(Return_Data_Type == Return_Position)//??????????
            {
                if(tilt_encoder_type == Encoder_AS5600)
                {
                    send_data[4] = ((u16)(Tilt_Angle_U2 * Tilt_Encoder_Range / Circle_Value)) >> 8;
                    send_data[5] = ((u16)(Tilt_Angle_U2 * Tilt_Encoder_Range / Circle_Value)) & 0xff;
                    send_data[6] = Check_Sum(send_data,1,6);
                    send_len = 7;
                }
                else
                {
                    send_data[4] = ((u32)(Tilt_Angle_U2 * Tilt_Encoder_Range / Circle_Value)) >> 16;
                    send_data[5] = ((u32)(Tilt_Angle_U2 * Tilt_Encoder_Range / Circle_Value)) >> 8;
                    send_data[6] = ((u32)(Tilt_Angle_U2 * Tilt_Encoder_Range / Circle_Value)) & 0xff;
                    send_data[7] = Check_Sum(send_data,1,7);
                    send_len = 8;
                }
            }
            else//?????
            {
                if(Angle_Accuracy_Flag)//?????Э??
                {
                    send_data[4] = ((u32)(Tilt_Angle_U2 * 1000)) >> 16;
                    send_data[5] = ((u16)(Tilt_Angle_U2 * 1000)) >> 8;
                    send_data[6] = ((u16)(Tilt_Angle_U2 * 1000)) & 0xff;
                    send_data[7] = Check_Sum(send_data,1,7);
                    send_len = 8;
                }
                else
                {
                    send_data[4] = ((u16)(Tilt_Angle_U2 * 100)) >> 8;
                    send_data[5] = ((u16)(Tilt_Angle_U2 * 100)) & 0xff;
                    send_data[6] = Check_Sum(send_data,1,6);
                    send_len = 7;
                }
//                #endif
            }
            USART_x = 2;
//            Post_Information_Function_TX(Sem_USART_TX,&USART_TX_BOX,USART_x,send_data,send_len);
            break;
        case Query_ZOOM:
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
                USART_x = 2;                                          //20220610 ?? ???????????zoom???????12????
            }
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
                USART_x = 2;                                           //20220610 ?? ???????????zoom???????12????
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
                USART_x = 2;
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
            USART_x = 2;
            break;
        case Query_Type_PT:
            send_data[2] = 0x28;
            send_data[3] = 0x18;
            send_data[4] = PT_Type >> 8;
            send_data[5] = PT_Type & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
            USART_x = 2;
            break;
        case Query_Time_Current:
            send_data[2] = 0x28;
            send_data[3] = 0x19;
            send_data[4] = TimeAndData >> 8;
            send_data[5] = TimeAndData & 0xff;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
            USART_x = 2;
            break;
        case Query_Encoder_PT:
            send_data[2] = 0x28;
            send_data[3] = 0x1a;
            send_data[4] = pan_encoder_type;
            send_data[5] = tilt_encoder_type;
            send_data[6] = Check_Sum(send_data,1,6);
            send_len = 7;
            USART_x = 2;
            break;
				case Query_Speed_Pan_Max:
						send_data[2] =0xa6;
						send_data[3] =0x15;
						send_data[4] = (Pan_A_MaxSpeed&0xff00)>>8;
						send_data[5] = Pan_A_MaxSpeed&0xff;
						send_data[6] = Check_Sum(send_data,1,6);
						send_len = 7;
						USART_x = 2;break;
				case Query_Speed_Pan_Min:
						send_data[2] =0xa6;
						send_data[3] =0x17;
						send_data[4] = (Pan_A_MinSpeed&0xff00)>>8;
						send_data[5] = Pan_A_MinSpeed&0xff;
						send_data[6] = Check_Sum(send_data,1,6);
						send_len = 7;
						USART_x = 2;break;
				case Query_Speed_Tilt_Max:
						send_data[2] =0xa6;
						send_data[3] =0x19;
						send_data[4] = (Tilt_A_MaxSpeed&0xff00)>>8;
						send_data[5] = Tilt_A_MaxSpeed&0xff;
						send_data[6] = Check_Sum(send_data,1,6);
						send_len = 7;
						USART_x = 2;break;
				case Query_Speed_Tilt_Min:
						send_data[2] =0xa6;
						send_data[3] =0x1b;
						send_data[4] = (Tilt_A_MinSpeed&0xff00)>>8;
						send_data[5] = Tilt_A_MinSpeed&0xff;
						send_data[6] = Check_Sum(send_data,1,6);
						send_len = 7;
						USART_x = 2;break;
						
            //****************************************************************
            //??????
            //特殊功能
        case Origin_Machine:            //设置机器原点
        {
            if (Down_Data == 0x0001)
            {
                memcpy(PT_Position_Record_U2,Position_Data_Format_U2,8);
                if(pan_encoder_type == 1)
                {
                    //              PT_Position_Record[4] = (u16)(Pan_Real_Angle_1 * 100) >> 8;
                    //              PT_Position_Record[5] = (u16)(Pan_Real_Angle_1 * 100) & 0xff;
                    //              Pan_Machine_Origin_Angle = Pan_Real_Angle_1;
                }
                else if(pan_encoder_type == 2)
                {
                    PT_Position_Record_U2[4] = (Pan_Encoder_Data >> 8)  & 0xff;
                    PT_Position_Record_U2[5] = Pan_Encoder_Data & 0xff;
                    Pan_Machine_Origin = Pan_Encoder_Data;
                }
                else if(pan_encoder_type == 3 || pan_encoder_type == 4)
                {
                    PT_Position_Record_U2[2] = (Pan_Encoder_Position_Real >> 16) & 0xff;
                    PT_Position_Record_U2[3] = (Pan_Encoder_Position_Real >> 8) & 0xff;
                    PT_Position_Record_U2[4] = Pan_Encoder_Position_Real & 0xff;
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
                    PT_Position_Record_U2[6] = (Tilt_Encoder_Data >> 8)  & 0xff;
                    PT_Position_Record_U2[7] = Tilt_Encoder_Data & 0xff;
                    Tilt_Machine_Origin = Tilt_Encoder_Data;
                    Origin_Machine_Flag = 5;
                }
                else if(tilt_encoder_type == 3 || tilt_encoder_type == 4)
                {
                    PT_Position_Record_U2[5] = (Tilt_Encoder_Position_Real >> 16) & 0xff;
                    PT_Position_Record_U2[6] = (Tilt_Encoder_Position_Real >> 8) & 0xff;
                    PT_Position_Record_U2[7] = Tilt_Encoder_Position_Real & 0xff;
                    Tilt_Machine_Origin = Tilt_Encoder_Position_Real;
                    Origin_Machine_Flag = 5;
                    tilt_track_offset = 0;
                }
                else
                {
                    Tilt_Machine_Origin = 0;
                }
                EEPRom_WriteDataLen(EEPROM_Parameter,Machine_Origin,PT_Position_Record_U2,Parameter_Bytes_NUN);
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
                    EEPRom_WriteDataLen(EEPROM_Preset,i,ERASE_EE_Preset_U2,Preset_Bytes_NUN);
                    //*****************************
                    //喂狗
                    IWDG_ReloadCounter();
                }
                for(j = 1; j < EE_TYPE / Parameter_Bytes_NUN; j ++)
                {
                    EEPRom_WriteDataLen(EEPROM_Parameter,j,ERASE_EE_Parameter_U2,Parameter_Bytes_NUN);
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
                USART_x = 1;
            }
            else
            {
                if(((double)Down_Data / 100) < Tilt_Angle_Limit_Up)
                {
                    Tilt_Limit_Up = ((double)Down_Data) / 100;
                    PT_Position_Record_U2[1] = 0;
                    PT_Position_Record_U2[2] = 0;
                    PT_Position_Record_U2[3] = 0;
                    PT_Position_Record_U2[4] = 0;
                    PT_Position_Record_U2[5] = 0;
                    PT_Position_Record_U2[6] = Down_Data >> 8;
                    PT_Position_Record_U2[7] = Down_Data & 0xff;
                    EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Up_EE,PT_Position_Record_U2,Parameter_Bytes_NUN);
                }
                else
                {
                    Tilt_Limit_Up = Tilt_Angle_Limit_Up;
                    PT_Position_Record_U2[1] = 0;
                    PT_Position_Record_U2[2] = 0;
                    PT_Position_Record_U2[3] = 0;
                    PT_Position_Record_U2[4] = 0;
                    PT_Position_Record_U2[5] = 0;
                    PT_Position_Record_U2[6] = (u16)(Tilt_Limit_Up * 100) >> 8;
                    PT_Position_Record_U2[7] = (u16)(Tilt_Limit_Up * 100) & 0xff;
                    EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Up_EE,PT_Position_Record_U2,Parameter_Bytes_NUN);
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
                USART_x = 1;
            }
            else
            {
                if((((double)Down_Data / 100) < 360) && (((double)Down_Data / 100) > Tilt_Angle_Limit_Down))
                {
                    Tilt_Limit_Down = ((double)Down_Data) / 100;
                    PT_Position_Record_U2[1] = 0;
                    PT_Position_Record_U2[2] = 0;
                    PT_Position_Record_U2[3] = 0;
                    PT_Position_Record_U2[4] = 0;
                    PT_Position_Record_U2[5] = 0;
                    PT_Position_Record_U2[6] = Down_Data >> 8;
                    PT_Position_Record_U2[7] = Down_Data & 0xff;
                    EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Down_EE,PT_Position_Record_U2,Parameter_Bytes_NUN);
                }
                else
                {
                    Tilt_Limit_Down = Tilt_Angle_Limit_Down;
                    PT_Position_Record_U2[1] = 0;
                    PT_Position_Record_U2[2] = 0;
                    PT_Position_Record_U2[3] = 0;
                    PT_Position_Record_U2[4] = 0;
                    PT_Position_Record_U2[5] = 0;
                    PT_Position_Record_U2[6] = (u16)(Tilt_Limit_Down * 100) >> 8;
                    PT_Position_Record_U2[7] = (u16)(Tilt_Limit_Down * 100) & 0xff;
                    EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Machine_Down_EE,PT_Position_Record_U2,Parameter_Bytes_NUN);
                }
            }
            break;
        case Tilt_Angle_Direction://俯仰反转指令
            if((Down_Data & 0xff) == 0xff)
            {
                send_data[5] = Tilt_Encoder_Direction;
                send_data[6] = Check_Sum(send_data,1,6);
                send_len = 7;
                USART_x = 1;
            }
            else
            {
                Tilt_Encoder_Direction = Down_Data;
                memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
                PT_Position_Record_U2[7] = Tilt_Encoder_Direction;
                EEPRom_WriteDataLen(EEPROM_Parameter,Tilt_Angle_Direction_EE,PT_Position_Record_U2,Parameter_Bytes_NUN);
            }
            break;
        case Set_PID_Pan://水平PID调试
            Pan_Kp = (Down_Data >> 8);
            Pan_Ki = (Down_Data & 0xff);
            PT_Position_Record_U2[5] = Pan_Kd;
            PT_Position_Record_U2[6] = Pan_Ki;
            PT_Position_Record_U2[7] = Pan_Kp;
            EEPRom_WriteDataLen(EEPROM_Parameter, PID_Manual, PT_Position_Record_U2, Parameter_Bytes_NUN);
            Pan_Kp = Pan_Kp / 100;
            Pan_Ki = Pan_Ki / 100;
            break;
        case Set_PID_Tilt://俯仰PID调试
            Tilt_Kp = (Down_Data >> 8);
            Tilt_Ki = (Down_Data & 0xff);
            PT_Position_Record_U2[2] = Tilt_Kd;
            PT_Position_Record_U2[3] = Tilt_Ki;
            PT_Position_Record_U2[4] = Tilt_Kp;
            EEPRom_WriteDataLen(EEPROM_Parameter, PID_Manual, PT_Position_Record_U2, Parameter_Bytes_NUN);
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
            PT_Position_Record_U2[4] = pan_data.Main_Switch;
            PT_Position_Record_U2[5] = tilt_data.Main_Switch;
            PT_Position_Record_U2[6] = pan_data.length;
            PT_Position_Record_U2[7] = tilt_data.length;
            EEPRom_WriteDataLen(EEPROM_Parameter, Filter_EE, PT_Position_Record_U2, Parameter_Bytes_NUN);
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
        }
        case Adaptation:                                                                                //电机速度自检
        {
            watcher_pause_state = watcher_pause;    //守望位暂停状态改为‘暂停’
            Pan_Motion_Mode = Rocker_Motion;
            Tilt_Motion_Mode = Rocker_Motion;
            Tilt_Target_Speed = 0;
            Pan_Target_Speed = 0;
            if(PWM_P == 0 && PWM_T == 0)
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
                USART_x = 2;
            }
        }break;

        case Speed_Set:                                                                                 //速度比例设置
        {
            switch(Down_Data)
            {
                //修改水平电机比例
            case 0x0001:
                Speed_Scale_Pan=0.3;
                break;
            case 0x0002:
                Speed_Scale_Pan=0.6;
                break;
            case 0x0003:
                Speed_Scale_Pan=0.8;
                break;
            case 0x0101:
                Speed_Scale_Tilt=0.3;
                break;
            case 0x0102:
                Speed_Scale_Tilt=0.6;
                break;
            case 0x0103:
                Speed_Scale_Tilt=0.8;
                break;
            default:
                break;
            }
        }
        break;
//        case 0xff01://???????1??????
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
//        case 0xff02://???????2??????
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
//        case 0xff03://???????3??????
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
                Pan_Target_Angle = ((double)Down_Data) / Angle_Decimal + Pan_Origin_User;//??????
                if(Pan_Target_Angle > 360.0)
                {
                    Pan_Target_Angle = Pan_Target_Angle - 360.0;//??????
                }
                Pan_Location_Speed = Speed_Level_Pan(Down_CMD & 0xff00,0);
                Place_InOut_Pan = OutPlace;
                break;
            case Set_Angle_Tilt:
                if(Return_Data_Type == Return_Position)
                {
                    TTA = ((double)Down_Data) * Circle_Value / Tilt_Encoder_Range;  //Down_Data
                }
                else
                {
                    if(Tilt_Encoder_Direction == Encoder_Inversion)     //????????
                    {
                        TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                        if(TTA < 0)
                        {
                            TTA = Circle_Value + TTA;//??????
                        }
                    }
                    else
                    {
                        TTA = ((double)Down_Data) / Angle_Decimal + Tilt_Origin_User;//??????  zxz 20220927 ???????????????? TTA = ((double)Down_Data) / Angle_Decimal;
                        //TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                        if(TTA > Circle_Value)
                        {
                            TTA = TTA - Circle_Value;//??????
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
                break;
            case Preset_Extract:
                //if((Down_Data < 200 && Down_Data != 105 && Down_Data !=111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))
                if( ( Down_Data < 200 ) || ( Down_Data > 256 && Down_Data < 2000 ) )
                {
                    Stay_Time = 0;
                    if(EEPRom_ReadOneByte(EEPROM_Preset,(Down_Data - 1) * Preset_Bytes_NUN) == 0xaa)
                    {
                        EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,PT_Position_Extract_U2,Preset_Bytes_NUN);
                        Preset_Empty = 0;
                        CAM_Zoom_Direct[4] = PT_Position_Extract_U2[8] >> 4;
                        CAM_Zoom_Direct[5] = PT_Position_Extract_U2[8] & 0xf;
                        CAM_Zoom_Direct[6] = PT_Position_Extract_U2[9] >> 4;
                        CAM_Zoom_Direct[7] = PT_Position_Extract_U2[9] & 0xf;
                        Zoom_Position = ((u16)PT_Position_Extract_U2[8] << 8) + PT_Position_Extract_U2[9];
                        data_p = 0;
                        data_p |= PT_Position_Extract_U2[10];
                        data_p <<= 8;
                        data_p |= PT_Position_Extract_U2[11];
                        data_p <<= 8;
                        data_p |= PT_Position_Extract_U2[12];
                        data_t |= PT_Position_Extract_U2[13];
                        data_t <<= 8;
                        data_t |= PT_Position_Extract_U2[14];
                        data_t <<= 8;
                        data_t |= PT_Position_Extract_U2[15];
                        Pan_Target_Angle = ((double)data_p) / 1000.0;
                        Tilt_Target_Angle = ((double)data_t) / 1000.0;
                        if(Special_Function == Tour_Scan_Disable)
                        {
                            Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;
                            Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
                        }
                        else
                        {
                            Pan_Location_Speed = Tour_Speed;
                            Tilt_Location_Speed = Tour_Speed;
                        }
                        Pan_Motion_Mode = Location_Motion;
                        Tilt_Motion_Mode = Location_Motion;
                        Place_InOut_Pan = OutPlace;
                        Place_InOut_Tilt = OutPlace;
                        Preset_Extract_Flag = 1;
                        Preset_Extract_num = Down_Data;
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
}
u8 PT_Scan1_U2,PT_Scan2_U2;
u8 Scan_Border1_U2[8],Scan_Border2_U2[8];
void USART2_Special_Preset_Set(u16 num)
{
    u16 i1 = 0;
    u32 data_p1 = 0,data_p2 = 0;
    switch(num)
    {
    case Origin_Pan:                  //201?????????
    {
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_Origin_User = Pan_Real_Angle;
        pan_data.Disposable_Switch = 1;
        EEPRom_WriteDataLen(EEPROM_Parameter,Origin_Pan,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Origin_Tilt:                 //202?????????
    {
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        Tilt_Origin_User = Tilt_Real_Angle;
        tilt_data.Disposable_Switch = 1;
        EEPRom_WriteDataLen(EEPROM_Parameter,Origin_Tilt,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Angle_Return_Mode:           //203???????
    {
//            if(Return_Mode != Return_Query_485)
//            {
        Return_Mode = Return_RealTime;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record_U2,Parameter_Bytes_NUN);
//            }
    }
    break;
    case Tour_Speed_Regulation:       //204??????????
    {
        if(Tour_Speed < Tour_Speed_Max)
        {
            Tour_Speed = Tour_Speed + Speed_Offset;
        }
        else
        {
            Tour_Speed = Tour_Speed_Max;
        }
        Pan_Location_Speed = Tour_Speed;
        Tilt_Location_Speed = Tour_Speed * 0.6;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Speed >> 8;
        PT_Position_Record_U2[7] = Tour_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_Speed_Level:            //205?????????
    {
        Tour_Speed = Tour_Speed_Max;
        Pan_Location_Speed = Tour_Speed;
        Tilt_Location_Speed = Tour_Speed * 0.6;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Speed >> 8;
        PT_Position_Record_U2[7] = Tour_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Regulation:    //206?????????????
    {
        if(Tour_Time < Tour_WaitTime_Max)
        {
            Tour_Time = Tour_Time + WaitTime_Offset;
        }
        else
        {
            Tour_Time = Tour_WaitTime_Max;
        }
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Time >> 8;
        PT_Position_Record_U2[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Level:         //207????????????
    {
        Tour_Time = Tour_WaitTime_Max;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Time >> 8;
        PT_Position_Record_U2[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Regulation: //208??????????????
    {
        if(Watcher_Time < Watcher_WaitTime_Max)
        {
            Watcher_Time = Watcher_Time + WaitTime_Offset;
        }
        else
        {
            Watcher_Time = Watcher_WaitTime_Max;
        }
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Watcher_Time >> 8;
        PT_Position_Record_U2[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Level:      //209?????????????
    {
        Watcher_Time = Watcher_WaitTime_Max;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Watcher_Time >> 8;
        PT_Position_Record_U2[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Regulation:       //210??????????
    {
        if(Scan_Speed < Scan_Speed_Max)
        {
            Scan_Speed = Scan_Speed + Speed_Offset;
        }
        else
        {
            Scan_Speed = Scan_Speed_Max;
        }
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Scan_Speed >> 8;
        PT_Position_Record_U2[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Level:            //211?????????
    {
        Scan_Speed = Scan_Speed_Max;
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Scan_Speed >> 8;
        PT_Position_Record_U2[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Return_Type:                 //213?????
    {
        Return_Data_Type = Return_Angle;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Return_Data_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,Return_Type,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
//        case Laser_Adjust_Switch_UNV:       //162?????????????,????
    case Laser_Adjust_Switch:         //214????????????
    {
        Axial = 1;
        Pan_Motion_Mode = Rocker_Motion;
        Tilt_Motion_Mode = Rocker_Motion;
        Pan_Target_Speed = 0;
        Tilt_Target_Speed = 0;
        Special_Function = Tour_Scan_Disable;
    }
    break;
    case USART3_Protocol_Type_Switch:       //221 U3?????VISCA
    {
        USART3_Protocol_Type = Protocol_VISCA;
        PT_Position_Record_U2[7] = USART3_Protocol_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Self_Adaption_Switch:        //222?????????
    {
        Self_Adaption = Self_Adaption_Enable;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Self_Adaption;
        EEPRom_WriteDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Home_Position:               //223??????????
    {
        Pan_Motion_Mode = Location_Motion;
        Tilt_Motion_Mode = Location_Motion;
        Pan_Target_Angle = Pan_Origin_User;
        Tilt_Target_Angle = Tilt_Origin_User;
        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;     //?????????????
        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
        Place_InOut_Pan = OutPlace;
        Place_InOut_Tilt = OutPlace;
    }
    break;
    case Limit_Pan:                   //225??????λ
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Enable;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Pan_User_Limit_Switch;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Limit_Right:                 //226????????λ
    {
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_User_Limit_R = Pan_Real_Angle;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Right,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Limit_Left:                  //227????????λ
    {
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[5] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[6] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[7] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_User_Limit_L = Pan_Real_Angle;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Left,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228????????λ??????
    {
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        PT_Position_Record_U2[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        Watcher_Angle_Pan = Pan_Real_Angle;
        Watcher_Angle_Tilt = Tilt_Real_Angle;
        Watcher_Switch = ON;
        watcher_pause_state = watcher_continue; //????λ????????????????
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_Set,PT_Position_Record_U2,Parameter_Bytes_NUN);    //???????λ????
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[2] = 0x00;
        PT_Position_Record_U2[3] = 0x00;
        PT_Position_Record_U2[4] = 0x00;
        PT_Position_Record_U2[5] = 0x00;
        PT_Position_Record_U2[6] = 0x00;
        PT_Position_Record_U2[7] = 0x00;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record_U2,Parameter_Bytes_NUN); //??????λ??????
    }
    break;
    case Delete_Data:                 //????3??230?????????????λ???????????????λ???????????λ
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
            for(i1 = 1; i1 < (EE_TYPE / Preset_Bytes_NUN); i1 ++)   //eetype????????????eeprom???????????????λ????洢16?????
            {
                EEPRom_WriteDataLen(EEPROM_Preset,i1,ERASE_EE_Preset_U2,Preset_Bytes_NUN);  //EEPROM_Preset????????λ?洢??EEPROM
                //*****************************
                //ι??
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            if(pan_encoder_type == 3 && tilt_encoder_type == 3) //???????????ICMU?????????????????????
            {
                STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
            }
            delay_us(10000);
            SoftReset();        //???λ
        }
    }
    break;
    case Section_Border1:             //231???????????1
    {
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        PT_Position_Record_U2[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Section_Border1,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Section_Border2:             //232???????????2
    {
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        PT_Position_Record_U2[5] = (u32)(Tilt_Real_Angle * 1000) >> 16;
        PT_Position_Record_U2[6] = (u32)(Tilt_Real_Angle * 1000) >> 8;
        PT_Position_Record_U2[7] = (u32)(Tilt_Real_Angle * 1000) & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Section_Border2,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Switch:                 //233??????????
    {
        PT_Scan1_U2 = EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border1 - 1) * Parameter_Bytes_NUN);
        PT_Scan2_U2= EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border2 - 1) * Parameter_Bytes_NUN);
        if(PT_Scan1_U2 == 0xaa && PT_Scan2_U2 == 0xaa)
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border1,Scan_Border1_U2,Parameter_Bytes_NUN);
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border2,Scan_Border2_U2,Parameter_Bytes_NUN);
            if(EEPRom_ReadOneByte(EEPROM_Parameter,(Scan_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)
            {
                EEPRom_ReadDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Extract_U2,Parameter_Bytes_NUN);
                Scan_Speed = ((PT_Position_Extract_U2[6] | 0x0000) << 8) + PT_Position_Extract_U2[7];
            }
            else
            {
                Scan_Speed = Scan_Speed_Mid;
            }
            data_p1 = 0;
            data_p1 |= Scan_Border1_U2[2];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1_U2[3];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1_U2[4];
            data_p2 = 0;
            data_p2 |= Scan_Border2_U2[5];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2_U2[6];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2_U2[7];
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
    case Scan_Switch_1:                 //234???????180?????????
    {
        PT_Scan1_U2 = EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border1 - 1) * Parameter_Bytes_NUN);
        PT_Scan2_U2 = EEPRom_ReadOneByte(EEPROM_Parameter,(Section_Border2 - 1) * Parameter_Bytes_NUN);
        if(PT_Scan1_U2 == 0xaa && PT_Scan2_U2 == 0xaa)
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border1,Scan_Border1_U2,Parameter_Bytes_NUN);
            EEPRom_ReadDataLen(EEPROM_Parameter,Section_Border2,Scan_Border2_U2,Parameter_Bytes_NUN);
            if(EEPRom_ReadOneByte(EEPROM_Parameter,(Scan_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)
            {
                EEPRom_ReadDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Extract_U2,Parameter_Bytes_NUN);
                Scan_Speed = ((PT_Position_Extract_U2[6] | 0x0000) << 8) + PT_Position_Extract_U2[7];
            }
            else
            {
                Scan_Speed = Scan_Speed_Mid;
            }
            data_p1 = 0;
            data_p1 |= Scan_Border1_U2[2];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1_U2[3];
            data_p1 <<= 8;
            data_p1 |= Scan_Border1_U2[4];
            data_p2 = 0;
            data_p2 |= Scan_Border2_U2[5];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2_U2[6];
            data_p2 <<= 8;
            data_p2 |= Scan_Border2_U2[7];
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
        break;              //235???????汾
////***********************************************************************************************
////jing210714
//      case Tilt_Inversion:                    //249调整垂直角度增长方向
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
void USART2_Special_Preset_Delete(u16 num)
{
    switch(num)
    {
    case Origin_Pan:                  //201????????
    {
        Pan_Origin_User = 0;
        EEPRom_WriteOneByte(EEPROM_Parameter,(Origin_Pan - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Origin_Tilt:                 //202?????????
    {
        Tilt_Origin_User = 0;
        EEPRom_WriteOneByte(EEPROM_Parameter,(Origin_Tilt - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Tour_Speed_Level:            //205???????е?
    {
        Tour_Speed = Tour_Speed_Mid;
        Pan_Location_Speed = Tour_Speed;
        Tilt_Location_Speed = Tour_Speed * 0.6;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Speed >> 8;
        PT_Position_Record_U2[7] = Tour_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Level:         //207??????????е?
    {
        Tour_Time = Tour_WaitTime_Mid;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Time >> 8;
        PT_Position_Record_U2[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Level:      //209???????????е?
    {
        Watcher_Time = Watcher_WaitTime_Mid;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Watcher_Time >> 8;
        PT_Position_Record_U2[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Level:            //211???????е?
    {
        Scan_Speed = Scan_Speed_Mid;
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Scan_Speed >> 8;
        PT_Position_Record_U2[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228???????λ
    {
        memcpy(PT_Position_Record_U2,ERASE_EE_Parameter_U2,Parameter_Bytes_NUN);
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record_U2,Parameter_Bytes_NUN); //??????λ???????
        Watcher_Switch = OFF;
        watcher_pause_state = watcher_pause;    //????λ????????????????
    }
    break;
    case Section_Border1:             //231??????????1
    {
        EEPRom_WriteOneByte(EEPROM_Parameter,(Section_Border1 - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Section_Border2:             //232??????????2
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
u32 tttt_U2 = 0;
void USART2_Special_Preset_Extract(u16 num)
{
    //Q_ERR err_q;
    u16 i3 = 0,i1 = 0,i2 = 0;
    u32 data_p = 0,data_t = 0;
    switch(num)
    {
    case Angle_Return_Mode:           //203????????
    {
        Return_Mode = Return_Query;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_Speed_Regulation:       //204?????????
    {
        if(Tour_Speed > Tour_Speed_Min)
        {
            Tour_Speed = Tour_Speed - Speed_Offset;
        }
        else
        {
            Tour_Speed = Tour_Speed_Min;
        }
        Pan_Location_Speed = Tour_Speed;
        Tilt_Location_Speed = Tour_Speed * 0.6;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Speed >> 8;
        PT_Position_Record_U2[7] = Tour_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_Speed_Level:            //205????????С
    {
        Tour_Speed = Tour_Speed_Min;
        Pan_Location_Speed = Tour_Speed;
        Tilt_Location_Speed = Tour_Speed * 0.6;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Speed >> 8;
        PT_Position_Record_U2[7] = Tour_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Regulation:    //206????????????
    {
        if(Tour_Time > Tour_WaitTime_Min)
        {
            Tour_Time = Tour_Time - WaitTime_Offset;
        }
        else
        {
            Tour_Time = Tour_WaitTime_Min;
        }
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Time >> 8;
        PT_Position_Record_U2[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Tour_StayTime_Level:         //207???????????С
    {
        Tour_Time = Tour_WaitTime_Min;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Tour_Time >> 8;
        PT_Position_Record_U2[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Regulation: //208?????????????
    {
        if(Watcher_Time > Watcher_WaitTime_Min)
        {
            Watcher_Time = Watcher_Time - WaitTime_Offset;
        }
        else
        {
            Watcher_Time = Watcher_WaitTime_Min;
        }
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Watcher_Time >> 8;
        PT_Position_Record_U2[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Level:      //209????????????С
    {
        Watcher_Time = Watcher_WaitTime_Min;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Watcher_Time >> 8;
        PT_Position_Record_U2[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Regulation:       //210?????????
    {
        if(Scan_Speed > Scan_Speed_Min)
        {
            Scan_Speed = Scan_Speed - Speed_Offset;
        }
        else
        {
            Scan_Speed = Scan_Speed_Min;
        }
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Scan_Speed >> 8;
        PT_Position_Record_U2[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Level:            //211????????С
    {
        Scan_Speed = Scan_Speed_Min;
        Pan_Location_Speed = Scan_Speed;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[6] = Scan_Speed >> 8;
        PT_Position_Record_U2[7] = Scan_Speed & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Scan_Speed_Level,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Return_Query_485:            //212??485??
    {
        Return_Mode = Return_Query_485;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Return_Type:                 //213??????
    {
        Return_Data_Type = Return_Position;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Return_Data_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,Return_Type,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
//        case Laser_Adjust_Switch_UNV:       //162?????????????,????
    case Laser_Adjust_Switch:         //214?????????????
    {
        Axial = 0;
//            Post_Information_Function(Sem_USART3_TX,&USART3_TX_BOX,Preset_Extract,Laser_Adjust_Switch);
    }
    break;
    case USART3_Protocol_Type_Switch:       //221 U3?????PELCO
    {
        USART3_Protocol_Type = Protocol_PELCO_D;
        PT_Position_Record_U2[7] = USART3_Protocol_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Self_Adaption_Switch:        //222????????
    {
        Self_Adaption = Self_Adaption_Disable;
        memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Self_Adaption;
        EEPRom_WriteDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Home_Position:               //223??180??????????
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
        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;     //?????????????
        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
        Place_InOut_Pan = OutPlace;
        Place_InOut_Tilt = OutPlace;
    }
    break;
    case Limit_Pan:                   //225???????λ
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Disable;
        memcpy(PT_Position_Record_U2,PT_Position_Extract_U2,Parameter_Bytes_NUN);
        PT_Position_Record_U2[7] = Pan_User_Limit_Switch;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Record_U2,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228????????λ
    {
        if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_Set - 1) * Parameter_Bytes_NUN) == 0xaa)//228???????λ
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,Watcher_Set,PT_Position_Extract_U2,Parameter_Bytes_NUN);
            data_p = 0;
            data_p |= PT_Position_Extract_U2[2];
            data_p <<= 8;
            data_p |= PT_Position_Extract_U2[3];
            data_p <<= 8;
            data_p |= PT_Position_Extract_U2[4];
            data_t = 0;
            data_t |= PT_Position_Extract_U2[5];
            data_t <<= 8;
            data_t |= PT_Position_Extract_U2[6];
            data_t <<= 8;
            data_t |= PT_Position_Extract_U2[7];
            Watcher_Angle_Pan = ((double)data_p) / 1000.0;
            Watcher_Angle_Tilt = ((double)data_t) / 1000.0;
            memcpy(PT_Position_Record_U2,Position_Data_Format_U2,Parameter_Bytes_NUN);
            PT_Position_Record_U2[2] = 0x00;
            PT_Position_Record_U2[3] = 0x00;
            PT_Position_Record_U2[4] = 0x00;
            PT_Position_Record_U2[5] = 0x00;
            PT_Position_Record_U2[6] = 0x00;
            PT_Position_Record_U2[7] = 0x00;
            EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record_U2,Parameter_Bytes_NUN); //??????λ??????
            Watcher_Switch = ON;    //????????λ
            watcher_pause_state = watcher_continue;
        }
        else
        {
            Watcher_Switch = OFF;
            watcher_pause_state = watcher_pause;
        }
    }
    break;
    case Tour_Switch:                 //229???????
    {
        watcher_pause_state = watcher_pause;    //????λ???????????????
        Preset_Number = 0;
        Pan_Target_Speed = 0;
        Stay_Time = 0;
        Pan_Motion_Mode = Rocker_Motion;
        Tilt_Motion_Mode = Rocker_Motion;
        Tilt_Target_Speed = 0;
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
                EEPRom_ReadDataLen(EEPROM_Parameter,Tour_Speed_Level,PT_Position_Extract_U2,Parameter_Bytes_NUN);
                Tour_Speed = ((PT_Position_Extract_U2[6] | 0x0000) << 8) + PT_Position_Extract_U2[7];
            }
            else
            {
                Tour_Speed = Tour_Speed_Mid;
            }
            Pan_Location_Speed = Tour_Speed;
            Tilt_Location_Speed = Tour_Speed * 0.6;
            if(Preset_Number == 1)
            {
            }
            else
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
    break;
    case Delete_Data:                 //????3??230?????????????????????λ?????????λ
    {
        Delete_Data_FLag ++;
        if(Delete_Data_FLag == 3)
        {
            Axial = 0;
            Delete_Data_FLag = 0;
            USART1_DISABLE();
            TIM_SetCompare1(TIM9,0);
            TIM_SetCompare2(TIM9,0);
            for(i1 = 1; i1 < (EE_TYPE / sizeof(ERASE_EE_Preset_U2)); i1 ++)
            {
                EEPRom_WriteDataLen(EEPROM_Preset,i1,ERASE_EE_Preset_U2,sizeof(ERASE_EE_Preset_U2));
                //*****************************
                //ι??
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            for(i2 = 1; i2 < 256; i2 ++)
            {
                EEPRom_WriteDataLen(EEPROM_Parameter,i2,ERASE_EE_Parameter_U2,sizeof(ERASE_EE_Parameter_U2));
                //*****************************
                //ι??
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            if(pan_encoder_type == 3 && tilt_encoder_type == 3) //???????????ICMU????????????????????
            {
                STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
            }
            delay_us(10000);
            SoftReset();//????λ
        }
    }
    break;
    case Scan_Switch:                 //233??????????????裩
    {
        Special_Function = Tour_Scan_Disable;
    }
    break;
    case Query_Versions:
        break;        //235???????汾
////***********************************************************************************************
////jing210714
//      case Tilt_Inversion:                    //249调整垂直角度增长方向
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
u8 Origin_Machine_Data_U2[8];
