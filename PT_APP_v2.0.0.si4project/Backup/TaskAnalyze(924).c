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
																											//ª˘±æ≤Ÿ◊˜
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Direction_Stop:  
        Imitate_determine = 1;
        Pan_Motion_Mode = Rocker_Motion;                                    //…Ë÷√ÀÆ∆Ω“°∏Àƒ£ Ω
        Tilt_Motion_Mode = Rocker_Motion;                                   //…Ë÷√¥π÷±“°∏Àƒ£ Ω
        Pan_Target_Speed = 0;                                               //ÀÆ∆Ωƒø±ÍÀŸ∂»≥ı ºªØ
        Tilt_Target_Speed = 0;                                              //¥π÷±ƒø±ÍÀŸ∂»≥ı ºªØ
        watcher_pause_state = watcher_continue;                 // ÿÕ˚Œª‘›Õ£◊¥Ã¨∏ƒŒ™°ÆºÃ–¯°Ø
        Special_Function = Tour_Scan_Disable;                               //Õ£÷π—≤ºÏ
        Stay_Time = 0;
        if(USART3_Protocol_Type == Protocol_VISCA)                          //≈–∂œ–≠“È «∑ÒŒ™VISCA–≠“È
        {
            memcpy(send_data,CAM_Zoom_Stop,6);                              //∞¥–≠“Èƒ⁄»›ÃÓ≥‰
            send_len = 6;
        }
        USART_x = 3;//¥Æø⁄3
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
            Special_Function = Tour_Scan_Disable;                               //Õ£÷π—≤ºÏ
            Stay_Time = 0;                                                      //Õ£¡Ù ±º‰
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
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
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
        }
        break;
    case Direction_Left:
        if(!Axial)                                                                  //≈–∂œ «∑ÒŒ™Õ∏¥´ƒ£ Ω
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
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
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
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
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
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
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
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
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
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
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
            USART_x = 0;//≤ª∑¢ÀÕ
        }
        else
        {
            USART_x = 3;//¥Æø⁄3
        }
        break;
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//∏®÷˙ø™πÿ
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case AUX_Switch_ON:
    {
        USART_x = 3;//¥Æø⁄3
    }
    break;
    case AUX_Switch_OFF:
    {
        USART_x = 3;//¥Æø⁄3
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
//          º§π‚µ∆≤Ÿ◊˜
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
																											//‘§÷√Œªπ¶ƒ‹
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Preset_Set:
        if(Down_Data >= 1 && Tilt_Hard_Limit_FLAG == Tilt_Limit_None)
        {
            //if((Down_Data < 200 && Down_Data != 105 && Down_Data != 111 && Down_Data != 114 && Down_Data != 116) || (Down_Data > 256 && Down_Data < 2000))    //105,111,114,116Œ™º§π‚µ˜Ω⁄º‹µƒ‘§÷√Œª”…”⁄≤ª π”√À˘“‘◊¢ ÕµÙ
            if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //»°œ˚µÙ¡Àº§π‚µ∆‘§÷√Œª£¨±ÿ“™ ±ø…Ω´…œ√Êµƒ◊¢ Õ––ª÷∏¥£¨≤¢Ω´¥À––◊¢ Õ
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
        if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //»°œ˚µÙ¡Àº§π‚µ∆‘§÷√Œª£¨±ÿ“™ ±ø…Ω´…œ√Êµƒ◊¢ Õ––ª÷∏¥£¨≤¢Ω´¥À––◊¢ Õ
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
        if((Down_Data < 200 ) || (Down_Data > 256 && Down_Data < 2000)) //»°œ˚µÙ¡Àº§π‚µ∆‘§÷√Œª£¨±ÿ“™ ±ø…Ω´…œ√Êµƒ◊¢ Õ––ª÷∏¥£¨≤¢Ω´¥À––◊¢ Õ
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
            Pan_Target_Angle = ((double)Down_Data) / Angle_Decimal + Pan_Origin_User;//∏°µ„–Õ
        }
        if(Pan_Target_Angle > Circle_Value)
        {
            Pan_Target_Angle = Pan_Target_Angle - Circle_Value;//∏°µ„–Õ
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
            if(Tilt_Encoder_Direction == Encoder_Inversion)     //◊¯±Í∑¥œÚ¥¶¿Ì
            {
                TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                if(TTA < 0)
                {
                    TTA = Circle_Value + TTA;//∏°µ„–Õ
                }
            }
            else
            {
                TTA = ((double)Down_Data) / Angle_Decimal + Tilt_Origin_User;//∏°µ„–Õ
                //TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                if(TTA > Circle_Value)
                {
                    TTA = TTA - Circle_Value;//∏°µ„–Õ
                }
            }
        }
        if(!(TTA > Tilt_Limit_Up && TTA < Tilt_Limit_Down))
        {
            Stay_Time = 0;
            Tilt_Target_Angle = TTA;        //Ω´¡Ÿ ±º∆À„÷µ¥´µ›∏¯ƒø±ÍΩ«∂»
            Special_Function = Tour_Scan_Disable;
            Tilt_Motion_Mode = Location_Motion;
            Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
            Place_InOut_Tilt = OutPlace;
        }
        USART_x = 0;
        break;
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//≤È—Ø≤Ÿ◊˜
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Query_Angle_Pan:
        Pan_Angle = pan_angle_query;
        send_data[2] = 0x00;
        send_data[3] = 0x59;
        if(Return_Data_Type == Return_Position)//±‡¬Î∆˜÷µªÿ¥´
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
        else//Ω«∂»ªÿ¥´
        {
            if(Angle_Accuracy_Flag)//∏ﬂæ´∂»–≠“È
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
        if(Return_Data_Type == Return_Position)//±‡¬Î∆˜÷µªÿ¥´
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
        else//Ω«∂»ªÿ¥´
        {
            if(Angle_Accuracy_Flag)//∏ﬂæ´∂»–≠“È
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
																											//Ãÿ ‚π¶ƒ‹
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Origin_Machine:            //…Ë÷√ª˙∆˜‘≠µ„
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
    case Clear_Parameter_Presets:            //«Â≥˝”√ªß≤Œ ˝∫Õ‘§÷√Œª
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
                //Œππ∑
                IWDG_ReloadCounter();
            }
            for(j = 1; j < EE_TYPE / Parameter_Bytes_NUN; j ++)
            {
                EEPRom_WriteDataLen(EEPROM_Parameter,j,ERASE_EE_Parameter,Parameter_Bytes_NUN);
                //*****************************
                //Œππ∑
                IWDG_ReloadCounter();
            }
            delay_us(10000);
            SoftReset();
        }
    }
    break;
    case Clear_Flash:                //≤¡≥˝FLSH
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
    case Tilt_Angle_Direction://∏©—ˆ∑¥◊™÷∏¡Ó
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
    case Set_PID_Pan://ÀÆ∆ΩPIDµ˜ ‘
        Pan_Kp = (Down_Data >> 8);
        Pan_Ki = (Down_Data & 0xff);
        PT_Position_Record[5] = Pan_Kd;
        PT_Position_Record[6] = Pan_Ki;
        PT_Position_Record[7] = Pan_Kp;
        EEPRom_WriteDataLen(EEPROM_Parameter, PID_Manual, PT_Position_Record, Parameter_Bytes_NUN);
        Pan_Kp = Pan_Kp / 100;
        Pan_Ki = Pan_Ki / 100;
        break;
    case Set_PID_Tilt://∏©—ˆPIDµ˜ ‘
        Tilt_Kp = (Down_Data >> 8);
        Tilt_Ki = (Down_Data & 0xff);
        PT_Position_Record[2] = Tilt_Kd;
        PT_Position_Record[3] = Tilt_Ki;
        PT_Position_Record[4] = Tilt_Kp;
        EEPRom_WriteDataLen(EEPROM_Parameter, PID_Manual, PT_Position_Record, Parameter_Bytes_NUN);
        Tilt_Kp =   Tilt_Kp / 1000;
        Tilt_Ki = Tilt_Ki / 1000;
        break;
    case Setup_Filter://¬À≤®∆˜…Ë÷√
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
        Pan_Motion_Mode = Rocker_Motion;                                    //…Ë÷√ÀÆ∆Ω“°∏Àƒ£ Ω
        Tilt_Motion_Mode = Rocker_Motion;                                   //…Ë÷√¥π÷±“°∏Àƒ£ Ω
        Pan_Target_Speed = 0;                                               //ÀÆ∆Ωƒø±ÍÀŸ∂»≥ı ºªØ
        Tilt_Target_Speed = 0;                                              //¥π÷±ƒø±ÍÀŸ∂»≥ı ºªØ
        watcher_pause_state = watcher_continue;                 // ÿÕ˚Œª‘›Õ£◊¥Ã¨∏ƒŒ™°ÆºÃ–¯°Ø
        Special_Function = Tour_Scan_Disable;                               //Õ£÷π—≤ºÏ
        Stay_Time = 0;
        if(USART3_Protocol_Type == Protocol_VISCA)                          //≈–∂œ–≠“È «∑ÒŒ™VISCA–≠“È
        {
            memcpy(send_data,CAM_Zoom_Stop,6);                              //∞¥–≠“Èƒ⁄»›ÃÓ≥‰
            send_len = 6;
        }
        USART_x = 3;//¥Æø⁄3
        U2_switch = (Down_Data & 0xff);
    }break;
    case Adaptation:                                                                                //µÁª˙ÀŸ∂»◊‘ºÏ
    {
		watcher_pause_state = watcher_pause;    // ÿÕ˚Œª‘›Õ£◊¥Ã¨∏ƒŒ™°Æ‘›Õ£°Ø
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
                Speed_Set_Satus = 1;//ÀŸ∂»…Ë÷√±Í÷æŒª

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
    case Speed_Set:                                                                                 //ÀŸ∂»±»¿˝…Ë÷√
    {
        Speed_Scale_Pan = ((Down_Data >> 8) > 100) ? 1.0f : (float)(Down_Data >> 8) / 100.0f;
        Speed_Scale_Tilt = ((Down_Data & 0xff) > 100) ? 1.0f : (float)(Down_Data & 0xff) / 100.0f;
        PT_Position_Record[11] = ((Down_Data >> 8) > 100) ? 0x64 : (float)(Down_Data >> 8);
        PT_Position_Record[12] = ((Down_Data & 0xff) > 100) ? 0x64 : (float)(Down_Data & 0xff);
        EEPRom_WriteDataLen(EEPROM_Parameter,(Max_Speed),PT_Position_Record,Preset_Bytes_NUN);
    }
    break;
#ifdef ZS
    case Extract_Number_Require://≤È—Ø‘§÷√ŒªæÕŒª«Èøˆ
    
            if (Extract_Number)
            {
                send_len = 5;
                memcpy(send_data,Position_Arrival,send_len);
                send_data[3] = Extract_Number;
                USART_x = 3;
                Extract_Number = 0;
            }            
    break;
    case Return_Intime_Visca://visca µ ±ªÿ¥´
    
            //Pan_Current_Angle_User;
            Tilt_Current_Angle_User = 360 - Tilt_Current_Angle_User;
            send_data[0] = 0x81;
            send_data[1] = 0x01;
            send_data[2] = 0x09;
            send_data[3] = 0x79;
            
            send_data[4] = ((u16)(Pan_Current_Angle_User * 100)) >> 12;
            send_data[5] = (((u16)(Pan_Current_Angle_User * 100)) >> 8) &0x0f;
            send_data[6] = (((u16)(Pan_Current_Angle_User * 100)) &0xff) >> 4;
            send_data[7] = (((u16)(Pan_Current_Angle_User * 100)) &0xff)&0x0f;  //ÀÆ∆ΩÀƒŒªµ•∂¿
            
            send_data[8] = ((u16)(Tilt_Current_Angle_User * 100)) >> 12;
            send_data[9] = (((u16)(Tilt_Current_Angle_User * 100)) >> 8) &0x0f;
            send_data[10] = (((u16)(Tilt_Current_Angle_User * 100)) &0xff) >> 4;
            send_data[11] = (((u16)(Tilt_Current_Angle_User * 100)) &0xff)&0x0f; //¥π÷±ÀƒŒªµ•∂¿
            
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
						//¥¢¥Ê◊¯±Í
            Section_Border1_Angle = ((double)Sweep[scan_id].border_left);
            Section_Border2_Angle = ((double)Sweep[scan_id].border_right);
						// ±º‰
            Scan_Time_Left = Sweep[scan_id].time_left * 100;
            Scan_Time_Right = Sweep[scan_id].time_right * 100;
						//zoom÷µ
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
    }
    break;
    case Rotate_Speed:
    {
        if ((Down_Data >> 8) <= 0x3f)
        {
            Speed_Rotate = Speed_Level_Pan(Down_Data,Self_Adaption);
        }
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
    }break;
    case Patrol_Setup:
    {
        Cruise.dir = Down_Data >> 8;
        Cruise.speed = Down_Data & 0xff;
        Cruise.speed_p = Speed_Level_Pan(Cruise.speed << 8,0);
        Cruise.speed_t = Speed_Level_Tilt(Cruise.speed,0);
    }break;
    case Patrol_Gradient:
    {
        Cruise.grads = Down_Data / 100.0;
    }break;
    case Patrol_Config:
    {
        if ((Down_Data  >> 8) == 0x01)
        {
            switch(Down_Data & 0xff)
            {
                case 0:
                    Special_Function = Skin_Fruit_ON;
                    if (Cruise.status != pause)
                    {
                        Cruise.status = start;
                    }
                    else
                    {
                        Cruise.status = go_on;
                    }
                    Cruise.situation = 2;
                    break;
                case 1:
                    Special_Function = Tour_Scan_Disable;
                    Cruise.status = pause;
										Cruise.T_data_pause = Tilt_Real_Angle;
										Cruise.P_data_pause = Pan_Real_Angle;
										Pan_Motion_Mode = Rocker_Motion;                                    //ËÆæÁΩÆÊ∞¥Âπ≥ÊëáÊùÜÊ®°Âºè
										Tilt_Motion_Mode = Rocker_Motion;                                   //ËÆæÁΩÆÂûÇÁõ¥ÊëáÊùÜÊ®°Âºè
										Pan_Target_Speed = 0;                                               //Ê∞¥Âπ≥ÁõÆÊ†áÈÄüÂ∫¶ÂàùÂßãÂå?
										Tilt_Target_Speed = 0;                                              //ÂûÇÁõ¥ÁõÆÊ†áÈÄüÂ∫¶ÂàùÂßãÂå
                    break;
                default:
                    Special_Function = Tour_Scan_Disable;
								    Pan_Motion_Mode = Rocker_Motion;                                    //ËÆæÁΩÆÊ∞¥Âπ≥ÊëáÊùÜÊ®°Âºè
										Tilt_Motion_Mode = Rocker_Motion;                                   //ËÆæÁΩÆÂûÇÁõ¥ÊëáÊùÜÊ®°Âºè
										Pan_Target_Speed = 0;                                               //Ê∞¥Âπ≥ÁõÆÊ†áÈÄüÂ∫¶ÂàùÂßãÂåñ
										Tilt_Target_Speed = 0;                                              //ÂûÇÁõ¥ÁõÆÊ†áÈÄüÂ∫¶ÂàùÂßãÂåñ
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
		//ø™ ºœﬂ…®
		case Start_Scan:
		{
				u16 scan_id;
        scan_id = (Down_Data & 0xff);
				//œ»≈–∂œid «∑Ò∫œ¿Ì£¨»Áπ˚≤ª∫œ¿Ì‘Ú÷±Ω”ÕÀ≥ˆ
        if (scan_id < 6)
        {
					//œ»…Ë÷√ÀŸ∂»£¨‘⁄»•÷¥––id
					//»Áπ˚√ª”–øÃ“‚…Ë÷√ÀŸ∂»£¨‘ÚŒ™ƒ¨»œÀŸ∂»0F
					if(Down_Data & 0xff00)
					{
						Sweep[scan_id].speed = Speed_Level_Pan(0x0F00,Self_Adaption);
						PT_Position_Record[3] = Sweep[scan_id].speed;
						EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
					
					}
					else //Ω¯»Î‘ÚÀµ√˜”–ÀŸ∂»…Ë÷√£¨Ω´data÷–µƒÀŸ∂»Ω¯––¥Ê¥¢
					{
					  Sweep[scan_id].speed = Speed_Level_Pan(Down_Data,Self_Adaption);
            PT_Position_Record[3] = Sweep[scan_id].speed;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),PT_Position_Record,Preset_Bytes_NUN);
					}
					//÷¥––id
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
						//¥¢¥Ê◊¯±Í
            Section_Border1_Angle = ((double)Sweep[scan_id].border_left);
            Section_Border2_Angle = ((double)Sweep[scan_id].border_right);
						// ±º‰
            Scan_Time_Left = Sweep[scan_id].time_left * 100;
            Scan_Time_Right = Sweep[scan_id].time_right * 100;
						//zoom÷µ
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
		//ƒÊ ±’ÎÀÆ∆Ω–˝◊™ø™ º
		case Rotate_Contrary_Start:
		{	
				//–˝◊™«∞≈–∂œÀŸ∂»
				if(Speed_Rotate == 0)
        {
            Speed_Rotate = Pan_Ultimate_MaxSpeed/2;
        }
				Special_Function = Rotating;
        Leading_mod = 1;
		
		}break;
		//—ßœ∞π¶ƒ‹º«¬º
		case Imitate_Record:
		{
			for(Imi.Imitation_top=0;Imi.Imitation_top < Imitate_size;Imi.Imitation_top++)
			{
				Imi.Instruct_Data[Imi.Imitation_top] = 0;
			}
			// ◊œ»º«¬ºµ±«∞Œª÷√◊¯±Í£¨ÃÓ≥‰goto√¸¡Ó
			Imi.Imitation_mode = Imitate_ON;     //Ω´ƒ£∑¬ø™πÿ¥Úø™
			//ÃÓ≥‰goto√¸¡Ó£¨±£¥Êø™ º◊¯±Í
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
			//ø…“‘ø™ º—ßœ∞¡À
		}break;
		case Imitate_Stop:
		{
			Imi.Imitation_mode = Imitate_OFF;     //Ω´ƒ£∑¬ø™πÿπÿ±’£¨±£¥Ê∏’∏’ ˝æ›ÃÓ≥‰
		}break;
		//—ßœ∞¬∑æ∂÷¥––
		case Imitate_start:
		{
			Imi.Imitation_mode = Route_ON;
			Imi.Imitation_Statt_Mode=0;
			for(Imi.Imitation_top=0;Imi.Imitation_top < Imitate_size;Imi.Imitation_top++)
			{
				Imi.Instruct_Data[Imi.Imitation_top] = 0;
			}
		}
		//ÀÆπ˚∆§π¶ƒ‹
		case Skin_Direction_Set:
		{
			//ªÒ»°id∫≈
			Fruit.Id = Down_Data & 0xFF;
			//ªÒ»°∑ΩœÚ
			Fruit.Direction = Down_Data >>8;
			//¥ÊE∑Ω÷–
			PT_Position_Record[1] = Fruit.Id;
			PT_Position_Record[2] = Fruit.Direction;
			EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),PT_Position_Record,Preset_Bytes_NUN);
		}break;
		case Skin_UP_Set:
		{
			//ªÒ»°id∫≈
			Fruit.Id = Down_Data & 0xFF;
			//±£¥Êµ±«∞¥π÷±◊¯±Í£¨¥ÊŒ™…œœﬁŒª
			Fruit.Up = Tilt_Real_Angle;
			PT_Position_Record[3] = (u32)(Fruit.Up * 1000) >> 16;
      PT_Position_Record[4] = (u32)(Fruit.Up * 1000) >> 8;
      PT_Position_Record[5] = (u32)(Fruit.Up * 1000);
			EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),PT_Position_Record,Preset_Bytes_NUN);
		}break;
		case Skin_Down_Set:
		{
			//ªÒ»°id∫≈
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
//        case 0xff01://∏¸∏ƒ¥Æø⁄1≤®Ãÿ¬ 
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
//        case 0xff02://∏¸∏ƒ¥Æø⁄2≤®Ãÿ¬ 
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
//        case 0xff03://∏¸∏ƒ¥Æø⁄3≤®Ãÿ¬ 
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
            Pan_Target_Angle = ((double)Down_Data) / Angle_Decimal + Pan_Origin_User;//∏°µ„–Õ
            if(Pan_Target_Angle > 360.0)
            {
                Pan_Target_Angle = Pan_Target_Angle - 360.0;//∏°µ„–Õ
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
                if(Tilt_Encoder_Direction == Encoder_Inversion)     //◊¯±Í∑¥œÚ¥¶¿Ì
                {
                    TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                    if(TTA < 0)
                    {
                        TTA = Circle_Value + TTA;//∏°µ„–Õ
                    }
                }
                else
                {
                    TTA = ((double)Down_Data) / Angle_Decimal + Tilt_Origin_User;//∏°µ„–Õ  zxz 20220927 Ωµ∏√––»•µÙ∏ƒŒ™œ¬“ª–– TTA = ((double)Down_Data) / Angle_Decimal;
                    //TTA = Tilt_Origin_User - ((double)Down_Data) / Angle_Decimal;
                    if(TTA > Circle_Value)
                    {
                        TTA = TTA - Circle_Value;//∏°µ„–Õ
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
                            //ªÒ»°÷¥––id
                            Fruit.Id = Down_CMD >> 8;
                            //ªÒ»°ÀŸ∂»
#ifdef REVERSE_TILT
                    Fruit.Speed = ~Speed_Level_Tilt(Down_Data>>8,Self_Adaption) + 1;
#else
                    Fruit.Speed = Speed_Level_Tilt(Down_Data>>8,Self_Adaption);
#endif
                        //±£¥Ê“ª»¶¥π÷±∆´“∆Ω«∂»
                            if((Down_Data &0xff) == 0x00)
                            {
                                Fruit.Tilt_Dif = 5;
                            }
                            else
                            {
                                Fruit.Tilt_Dif = Down_Data &0xff;
                            }
        
                        //Ã·»°e∑Ω÷–µƒ ˝æ›£¨…œœ¬œﬁŒª∑ΩœÚ
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
//Ãÿ ‚‘§÷√Œª
u8 PT_Scan1,PT_Scan2;
u8 Scan_Border1[8],Scan_Border2[8];
void Special_Preset_Set(u16 num)
{
    u16 i1 = 0;
    u32 data_p1 = 0,data_p2 = 0;
    switch(num)
    {
    case Origin_Pan:                  //201…Ë÷√ÀÆ∆Ω‘≠µ„
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
    case Origin_Tilt:                 //202…Ë÷√¥π÷±‘≠µ„
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
    case Angle_Return_Mode:           //203 µ ±ªÿ¥´ƒ£ Ω
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
    case Tour_Speed_Regulation:       //204—≤ºÏÀŸ∂»‘ˆº”
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
    case Tour_Speed_Level:            //205—≤ºÏÀŸ∂»◊Ó¥Û
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
    case Tour_StayTime_Regulation:    //206—≤ºÏÕ£¡Ù ±º‰‘ˆº”
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
    case Tour_StayTime_Level:         //207—≤ºÏÕ£¡Ù ±º‰◊Ó¥Û
    {
        Tour_Time = Tour_WaitTime_Max;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Regulation: //208 ÿÕ˚Õ£¡Ù ±º‰‘ˆº”
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
    case Watcher_StayTime_Level:      //209 ÿÕ˚Õ£¡Ù ±º‰◊Ó¥Û
    {
        Watcher_Time = Watcher_WaitTime_Max;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Regulation:       //210…®√ËÀŸ∂»‘ˆº”
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
    case Scan_Speed_Level:            //211…®√ËÀŸ∂»◊Ó¥Û
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
    case Return_Type:                 //213Ω«∂»ªÿ¥´
    {
        Return_Data_Type = Return_Angle;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Data_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,Return_Type,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
//        case Laser_Adjust_Switch_UNV:       //162ÕÀ≥ˆº§π‚µ∆µ˜Ω⁄ƒ£ Ω,”Ó ”
    case Laser_Adjust_Switch:         //214Ω¯»Îº§π‚µ∆µ˜Ω⁄ƒ£ Ω
    {
        Axial = 1;
        Pan_Motion_Mode = Rocker_Motion;
        Tilt_Motion_Mode = Rocker_Motion;
        Pan_Target_Speed = 0;
        Tilt_Target_Speed = 0;
        Special_Function = Tour_Scan_Disable;
    }
    break;
//    case USART3_Protocol_Type_Switch:       //221 U3ƒ£ Ω∏ƒŒ™VISCA
//    {
//        USART3_Protocol_Type = Protocol_VISCA;
//        PT_Position_Record[7] = USART3_Protocol_Type;
//        EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record,Parameter_Bytes_NUN);
//    }
//    break;
    case Self_Adaption_Switch:        //222ø™∆Ù◊‘  ”¶
    {
        Self_Adaption = Self_Adaption_Enable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Self_Adaption;
        EEPRom_WriteDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Home_Position:               //223ÀÆ∆Ω¥π÷±ªÿ‘≠µ„
    {
        Pan_Motion_Mode = Location_Motion;
        Tilt_Motion_Mode = Location_Motion;
        Pan_Target_Angle = Pan_Origin_User;
        Tilt_Target_Angle = Tilt_Origin_User;
        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;     // π”√◊Ó¥ÛÀŸ∂»ªÿ‘≠µ„
        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
        Place_InOut_Pan = OutPlace;
        Place_InOut_Tilt = OutPlace;
    }
    break;
    case Limit_Pan:                   //225¥Úø™ÀÆ∆ΩœﬁŒª
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Enable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Pan_User_Limit_Switch;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Limit_Right:                 //226…Ë÷√”“œﬁŒª
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[3] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[4] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_User_Limit_R = Pan_Real_Angle;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Right,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Limit_Left:                  //227…Ë÷√◊ÛœﬁŒª
    {
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[5] = (u32)(Pan_Real_Angle * 1000) >> 16;
        PT_Position_Record[6] = (u32)(Pan_Real_Angle * 1000) >> 8;
        PT_Position_Record[7] = (u32)(Pan_Real_Angle * 1000) & 0xff;
        Pan_User_Limit_L = Pan_Real_Angle;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Left,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228…Ë÷√ ÿÕ˚Œª≤¢ø™∆Ù
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
        watcher_pause_state = watcher_continue; // ÿÕ˚Œª‘›Õ£◊¥Ã¨∏ƒŒ™°ÆºÃ–¯°Ø
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_Set,PT_Position_Record,Parameter_Bytes_NUN);   //º«¬º ÿÕ˚Œª◊¯±Í
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[2] = 0x00;
        PT_Position_Record[3] = 0x00;
        PT_Position_Record[4] = 0x00;
        PT_Position_Record[5] = 0x00;
        PT_Position_Record[6] = 0x00;
        PT_Position_Record[7] = 0x00;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record,Parameter_Bytes_NUN);    //Ω´ ÿÕ˚Œªø™πÿº«¬º
    }
    break;
    case Delete_Data:                 //…Ë÷√3¥Œ230«Â≥˝À˘”–”√ªß‘§÷√Œª£¨≤ª∞¸¿®Ãÿ ‚‘§÷√Œª£¨≤ª”∞œÏ ÿÕ˚Œª
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
            for(i1 = 1; i1 < (EE_TYPE / Preset_Bytes_NUN); i1 ++)   //eetype∂‘”¶µƒ «≤ªÕ¨–Õ∫≈eepromµƒ◊÷Ω⁄ ˝£¨√ø∏ˆ‘§÷√Œª–Ë“™¥Ê¥¢16∏ˆ◊÷Ω⁄
            {
                EEPRom_WriteDataLen(EEPROM_Preset,i1,ERASE_EE_Preset,Preset_Bytes_NUN); //EEPROM_Preset «”√ªß‘§÷√Œª¥Ê¥¢µƒEEPROM
                //*****************************
                //Œππ∑
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            if(pan_encoder_type == 3 && tilt_encoder_type == 3) //»Áπ˚±‡¬Î∆˜ «ICMU£¨‘Ú–Ë“™º«¬º±‡¬Î∆˜µƒµ±«∞÷µ
            {
                STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
            }
            delay_us(10000);
            SoftReset();        //»Ì∏¥Œª
        }
    }
    break;
    case Section_Border1:             //231…Ë÷√…»«¯±ﬂΩÁ1
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
    case Section_Border2:             //232…Ë÷√…»«¯±ﬂΩÁ2
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
    case Scan_Switch:                 //233∆Ù∂Ø…»«¯…®√Ë
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
    case Scan_Switch_1:                 //234∆Ù∂Ø¥Û”⁄180∂»…»«¯…®√Ë
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
        break;              //235≤È—Ø‘∆Ã®∞Ê±æ
////***********************************************************************************************
////jing210714
//      case Tilt_Inversion:                    //249Ë∞ÉÊï¥ÂûÇÁõ¥ËßíÂ∫¶Â¢ûÈïøÊñπÂêë
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
    case Origin_Pan:                  //201…æ≥˝ÀÆ∆Ω‘≠µ„
    {
        Pan_Origin_User = 0;
        EEPRom_WriteOneByte(EEPROM_Parameter,(Origin_Pan - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Origin_Tilt:                 //202…æ≥˝¥π÷±‘≠µ„
    {
        Tilt_Origin_User = 0;
        EEPRom_WriteOneByte(EEPROM_Parameter,(Origin_Tilt - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Tour_Speed_Level:            //205—≤ºÏÀŸ∂»÷–µ»
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
    case Tour_StayTime_Level:         //207—≤ºÏÕ£¡Ù ±º‰÷–µ»
    {
        Tour_Time = Tour_WaitTime_Mid;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Level:      //209 ÿÕ˚Õ£¡Ù ±º‰÷–µ»
    {
        Watcher_Time = Watcher_WaitTime_Mid;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Level:            //211…®√ËÀŸ∂»÷–µ»
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
    case Watcher_Set:                 //228«Â≥˝ ÿÕ˚Œª
    {
        memcpy(PT_Position_Record,ERASE_EE_Parameter,Parameter_Bytes_NUN);
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record,Parameter_Bytes_NUN);    //Ω´ ÿÕ˚Œªø™πÿ«Â≥˝
        Watcher_Switch = OFF;
        watcher_pause_state = watcher_pause;    // ÿÕ˚Œª‘›Õ£◊¥Ã¨∏ƒŒ™°ÆºÃ–¯°Ø
    }
    break;
    case Section_Border1:             //231…æ≥˝…»«¯±ﬂΩÁ1
    {
        EEPRom_WriteOneByte(EEPROM_Parameter,(Section_Border1 - 1) * Parameter_Bytes_NUN,0xff);
    }
    break;
    case Section_Border2:             //232…æ≥˝…»«¯±ﬂΩÁ2
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
    case Angle_Return_Mode:           //203≤È—Øªÿ¥´ƒ£ Ω
    {
        Return_Mode = Return_Query;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Tour_Speed_Regulation:       //204—≤ºÏÀŸ∂»ºı…Ÿ
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
    case Tour_Speed_Level:            //205—≤ºÏÀŸ∂»◊Ó–°
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
    case Tour_StayTime_Regulation:    //206—≤ºÏÕ£¡Ù ±º‰ºı…Ÿ
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
    case Tour_StayTime_Level:         //207—≤ºÏÕ£¡Ù ±º‰◊Ó–°
    {
        Tour_Time = Tour_WaitTime_Min;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Tour_Time >> 8;
        PT_Position_Record[7] = Tour_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_StayTime_Regulation: //208 ÿÕ˚Õ£¡Ù ±º‰ºı…Ÿ
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
    case Watcher_StayTime_Level:      //209 ÿÕ˚Õ£¡Ù ±º‰◊Ó–°
    {
        Watcher_Time = Watcher_WaitTime_Min;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[6] = Watcher_Time >> 8;
        PT_Position_Record[7] = Watcher_Time & 0xff;
        EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Scan_Speed_Regulation:       //210…®√ËÀŸ∂»ºı…Ÿ
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
    case Scan_Speed_Level:            //211…®√ËÀŸ∂»◊Ó–°
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
    case Return_Query_485:            //212µ•485ƒ£ Ω
    {
        Return_Mode = Return_Query_485;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Mode;
        EEPRom_WriteDataLen(EEPROM_Parameter,Angle_Return_Mode,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Return_Type:                 //213◊¯±Íªÿ¥´
    {
        Return_Data_Type = Return_Position;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Return_Data_Type;
        EEPRom_WriteDataLen(EEPROM_Parameter,Return_Type,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
//        case Laser_Adjust_Switch_UNV:       //162ÕÀ≥ˆº§π‚µ∆µ˜Ω⁄ƒ£ Ω,”Ó ”
    case Laser_Adjust_Switch:         //214ÕÀ≥ˆº§π‚µ∆µ˜Ω⁄ƒ£ Ω
    {
        Axial = 0;
//            Post_Information_Function(Sem_USART3_TX,&USART3_TX_BOX,Preset_Extract,Laser_Adjust_Switch);
    }
    break;
//    case USART3_Protocol_Type_Switch:       //221 U3ƒ£ Ω∏ƒŒ™PELCO
//    {
//        USART3_Protocol_Type = Protocol_PELCO_D;
//        PT_Position_Record[7] = USART3_Protocol_Type;
//        EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record,Parameter_Bytes_NUN);
//    }
//    break;
    case Self_Adaption_Switch:        //222πÿ±’◊‘  ”¶
    {
        Self_Adaption = Self_Adaption_Disable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Self_Adaption;
        EEPRom_WriteDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Home_Position:               //223ÀÆ∆Ω180£¨¥π÷±ªÿ‘≠µ„
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
        Pan_Location_Speed = Pan_Ultimate_MaxSpeed*Speed_Scale_Pan;     // π”√◊Ó¥ÛÀŸ∂»ªÿ‘≠µ„
        Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed*Speed_Scale_Tilt;
        Place_InOut_Pan = OutPlace;
        Place_InOut_Tilt = OutPlace;
    }
    break;
    case Limit_Pan:                   //225πÿ±’ÀÆ∆ΩœﬁŒª
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Disable;
        memcpy(PT_Position_Record,Position_Data_Format,Parameter_Bytes_NUN);
        PT_Position_Record[7] = Pan_User_Limit_Switch;
        EEPRom_WriteDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Record,Parameter_Bytes_NUN);
    }
    break;
    case Watcher_Set:                 //228ø™∆Ù ÿÕ˚Œª
    {
        if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_Set - 1) * Parameter_Bytes_NUN) == 0xaa)//228”√ªß ÿÕ˚Œª
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
            EEPRom_WriteDataLen(EEPROM_Parameter,Watcher_ON_OFF,PT_Position_Record,Parameter_Bytes_NUN);    //Ω´ ÿÕ˚Œªø™πÿº«¬º
            Watcher_Switch = ON;    //ø™∆Ù ÿÕ˚Œª
            watcher_pause_state = watcher_continue;
        }
        else
        {
            Watcher_Switch = OFF;
            watcher_pause_state = watcher_pause;
        }
    }
    break;
    case Tour_Switch:                 //229ø™∆Ù—≤ºÏ
    {
        watcher_pause_state = watcher_pause;    // ÿÕ˚Œª‘›Õ£◊¥Ã¨∏ƒŒ™°Æ‘›Õ£°Ø
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
    case Delete_Data:                 //µ˜”√3¥Œ230«Â≥˝À˘”– ˝æ›∞¸¿®Ãÿ ‚‘§÷√Œª£¨…æ≥˝ ÿÕ˚Œª
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
                //Œππ∑
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            for(i2 = 1; i2 < 256; i2 ++)
            {
                EEPRom_WriteDataLen(EEPROM_Parameter,i2,ERASE_EE_Parameter,sizeof(ERASE_EE_Parameter));
                //*****************************
                //Œππ∑
                IWDG_ReloadCounter();
                Stay_Time = 0;
            }
            if(pan_encoder_type == 3 && tilt_encoder_type == 3) //»Áπ˚±‡¬Î∆˜ «ICMU‘Ú–Ë“™º«¬º±‡¬Î∆˜µ±«∞ ˝÷µ
            {
                STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
            }
            delay_us(10000);
            SoftReset();//÷±Ω”∏¥Œª
        }
    }
    break;
    case Scan_Switch:                 //233Õ£÷πÃÿ ‚π¶ƒ‹£®—≤ºÏ£¨…®√Ë£©
    {
        Special_Function = Tour_Scan_Disable;
    }
    break;
    case Query_Versions:
        break;        //235≤È—ØæµÕ∑∞Ê±æ
////***********************************************************************************************
////jing210714
//      case Tilt_Inversion:                    //249Ë∞ÉÊï¥ÂûÇÁõ¥ËßíÂ∫¶Â¢ûÈïøÊñπÂêë
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
    //“‘œ¬‘⁄…œµÁ∫Û∂¡»°FLASH‘⁄…œ¥ŒµÙµÁ ±±£¥Êµƒ ˝æ›
    pwr_data_encoder[0] = STMFLASH_ReadWord(track_offect);      //¥”FLASH÷–∂¡»°ÀÆ∆Ω±‡¬Î∆˜ ˝æ›
    pwr_data_encoder[1] = STMFLASH_ReadWord(track_offect + 4);  //¥”FLASH÷–∂¡»°∏©—ˆ±‡¬Î∆˜ ˝æ›
    pwr_data_encoder[2] = STMFLASH_ReadWord(track_offect + 8);  //¥”FLASH÷–∂¡»°…œµÁ∫Û∂¡»°±‡¬Î∆˜¥Œ ˝£¨∏√¥Œ ˝ª·“Á≥ˆ
    pwr_data_encoder[3] = STMFLASH_ReadWord(track_offect + 12); //¥”FLASH÷–∂¡»°CRC–£—È÷µ
    STM32Sector_Erase(track_offect);    //∂¡»°ÕÍ±œµÙµÁ±£¥ÊµƒÀÆ∆Ω∫Õ¥π÷± ˝æ›∫Û∂‘∏√«¯”ÚΩ¯––≤¡≥˝
    if(CRC_16((u8*)pwr_data_encoder,12) != (u16)pwr_data_encoder[3])    //≈–∂œ–£—È÷µ «∑Ò’˝»∑
    {
        flash_data_crc = 1; //flash_data_crcŒ™1±Ì æCRC¥ÌŒÛ
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
            if(!flash_data_crc)     //»Áπ˚µÙµÁFLASH÷–±£¥Êµƒ ˝æ›ø…øø£¨º¥CRC—È÷§’˝»∑
            {
                pan_track_err = (int)(((pwr_data_encoder[0] & 0x3ffff) - Pan_Encoder_Data_Last) << 14);
                pan_track_err /= 16384;
                pan_track_offset = pan_track_err / 8192 + (pan_track_err % 8192) / 4096;
                send_data[0] = 0xaa;
                send_data[4] = 0xaa;
                send_data[8] = 0xaa;
            }
            else            //»Áπ˚µÙµÁ±£¥Êµƒ ˝æ›≤ªø…øø
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
        Tilt_Limit_Up = Tilt_Angle_Limit_Up_default;    //»Áπ˚Œ¥…Ë÷√œﬁŒªΩ«∂»£¨‘Ú∞¥’’ƒ¨»œœﬁŒªΩ«∂»Ω¯––…Ë÷√
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Machine_Down_EE - 1) * Parameter_Bytes_NUN) == 0xaa)//0x02ff
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Limit_Machine_Down_EE,PT_Position_Extract,Parameter_Bytes_NUN);
        Tilt_Limit_Down = (double)(((u16)PT_Position_Extract[6] << 8) + PT_Position_Extract[7]) / 100;
    }
    else
    {
        Tilt_Limit_Down = Tilt_Angle_Limit_Down_default;    //»Áπ˚Œ¥…Ë÷√œﬁŒªΩ«∂»‘Ú∞¥’’ƒ¨»œœﬁŒªΩ«∂»Ω¯––…Ë÷√
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Tilt_Angle_Direction_EE - 1) * Parameter_Bytes_NUN) == 0xaa)//0x03ff//∂¡≥ˆ∑¥◊™±Í÷æ
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Tilt_Angle_Direction_EE,PT_Position_Extract,Parameter_Bytes_NUN);
        Tilt_Encoder_Direction = PT_Position_Extract[7];
    }
    else
    {
        Tilt_Encoder_Direction = Encoder_Forward;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Pan - 1) * Parameter_Bytes_NUN) == 0xaa)  //200ÀÆ∆ΩœﬁŒªø™πÿ
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Limit_Pan,PT_Position_Extract,Parameter_Bytes_NUN);
        Pan_User_Limit_Switch = PT_Position_Extract[7];
    }
    else
    {
        Pan_User_Limit_Switch = Pan_User_Limit_Disable;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Origin_Pan - 1) * Parameter_Bytes_NUN) == 0xaa) //201ÀÆ∆Ω‘≠µ„
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Origin_Tilt - 1) * Parameter_Bytes_NUN) == 0xaa)    //202¥π÷±‘≠µ„
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
		    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Max_Speed - 1) * Preset_Bytes_NUN) == 0xaa)//◊Ó∏ﬂÀŸ∂»
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Angle_Return_Mode - 1) * Parameter_Bytes_NUN) == 0xaa)//203ªÿ¥´∑Ω Ω£¨≤È—Øor µ ±
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Tour_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//205—≤ºÏÀŸ∂»
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Tour_StayTime_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//207—≤ºÏÕ£¡Ù ±º‰
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Tour_StayTime_Level,PT_Position_Extract,Parameter_Bytes_NUN);
        Tour_Time = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
    }
    else
    {
        Tour_Time = Tour_WaitTime_Min;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_StayTime_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//209ªÿ ÿÕ˚Œªµ»¥˝ ±º‰
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Watcher_StayTime_Level,PT_Position_Extract,Parameter_Bytes_NUN);
        Watcher_Time = ((PT_Position_Extract[6] | 0x0000) << 8) + PT_Position_Extract[7];
    }
    else
    {
        Watcher_Time = Watcher_WaitTime_Mid;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Scan_Speed_Level - 1) * Parameter_Bytes_NUN) == 0xaa)//211…®√ËÀŸ∂»
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Angle_Return_Mode - 1) * Parameter_Bytes_NUN) == 0xaa)  //212µ•485ƒ£ Ω
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Return_Type - 1) * Parameter_Bytes_NUN) == 0xaa)//213ªÿ¥´ ˝æ›¿‡–Õ£¨Ω«∂»or±‡¬Î∆˜÷µ
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Return_Type,PT_Position_Extract,8);
        Return_Data_Type = PT_Position_Extract[7];
    }
    else
    {
        Return_Data_Type = Return_Angle;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Self_Adaption_Switch - 1) * Parameter_Bytes_NUN) == 0xaa)//222◊‘  ”¶ø™πÿ
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,Self_Adaption_Switch,PT_Position_Extract,Parameter_Bytes_NUN);
        Self_Adaption = PT_Position_Extract[7];
    }
    else
    {
        Self_Adaption = Self_Adaption_Disable;
    }
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Right - 1) * Parameter_Bytes_NUN) == 0xaa)//226”√ªß”“œﬁŒª
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Limit_Left - 1) * Parameter_Bytes_NUN) == 0xaa)//227”√ªß◊ÛœﬁŒª
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_Set - 1) * Parameter_Bytes_NUN) == 0xaa)//228”√ªß ÿÕ˚Œª
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Watcher_ON_OFF - 1) * Parameter_Bytes_NUN) == 0xaa)//228”√ªß ÿÕ˚Œª
    {
        Watcher_Switch = ON;
    }
    else
    {
        Watcher_Switch = OFF;
    }
//    if(EEPRom_ReadOneByte(EEPROM_Parameter,(USART3_Protocol_Type_Switch - 1) * Parameter_Bytes_NUN) == 0xaa)//U3–≠“È¿‡–Õ
//    {
//        EEPRom_ReadDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Extract,Parameter_Bytes_NUN);
//        USART3_Protocol_Type = PT_Position_Extract[7];
//    }
//    else
//    {
//        USART3_Protocol_Type = Protocol_VISCA;
//    }
    watcher_pause_state = watcher_continue; //Ω´ ÿÕ˚Œª‘›Õ£◊¥Ã¨…Ë÷√Œ™ºÃ–¯£¨»Áπ˚ ÿÕ˚Œª∆Ù∂Ø‘Ú…œµÁ∫Û ÿÕ˚Œª¥¶”⁄º§ªÓ◊¥Ã¨
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(PID_Manual - 1) * Parameter_Bytes_NUN) == 0xaa)//PID ≤Œ ˝
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
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(Filter_EE - 1) * Parameter_Bytes_NUN) == 0xaa)//PID ≤Œ ˝
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
        tilt_data.length = 10;//??????¯A??
    }
    return state;
}
//***********************************************
//µ»±» ˝¡–
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
