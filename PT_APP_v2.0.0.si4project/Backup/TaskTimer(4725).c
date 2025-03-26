#include "TaskStart.h"
#include "TaskTimer.h"
#include "TaskUSART_TX.h"
#include "SHT30.h"
#include "TaskCAN.h"
#include "hardware.h"
#include "includes.h"
#include "parameters.h"
#include "delay.h"
#include "laser.h"
#include "TaskEncoder.h"
#include "TaskMotor.h"
#include "24LCxx.h"
u16 Tim_Com = 0;
u16 Tim_Com2 = 0;
u8 send_len = 0;
u8 send_data[20] = {0xff,0x01};
u8 USART_x = 3;
u32 Encoder_SPI_CMD(u16 cmd,u32 data)
{
    CPU_SR_ALLOC();
    u8 i;
    u32 p_data = 0;
    u32 s_data = 0;
    OS_CRITICAL_ENTER();//Ω¯»Î¡ŸΩÁ«¯
    s_data = data;
    Pan_SSI_CS_L;
    Pan_SSI_CK_L;
    for(i = 0; i < 16; i ++)
    {
        if(cmd & 0x8000)
        {
            Pan_SSI_DO_H;
        }
        else
        {
            Pan_SSI_DO_L;
        }
        delay_us(1);
        cmd <<= 1;
        Pan_SSI_CK_H;
        delay_us(2);
        Pan_SSI_CK_L;
        delay_us(1);
    }
    Pan_SSI_CS_H;
    delay_us(1);
    Pan_SSI_CS_L;
    Pan_SSI_CK_L;
    delay_us(1);
    for(i = 0; i < 24; i ++)
    {
        if(s_data & 0x800000)
        {
            Pan_SSI_DO_H;
        }
        else
        {
            Pan_SSI_DO_L;
        }
        delay_us(1);
        s_data <<= 1;
        Pan_SSI_CK_H;
        delay_us(2);
        p_data <<= 1;
        if(Pan_SSI_DI)
        {
            p_data ++;
        }
        Pan_SSI_CK_L;
        delay_us(1);
    }
    Pan_SSI_CS_H;
    delay_us(1);
    if(((p_data >> 8) & 0xff) == 0x01)
    {
        s_data = data;
        p_data = 0;
        Pan_SSI_CS_L;
        Pan_SSI_CK_L;
        delay_us(1);
        for(i = 0; i < 24; i ++)
        {
            if(s_data & 0x800000)
            {
                Pan_SSI_DO_H;
            }
            else
            {
                Pan_SSI_DO_L;
            }
            delay_us(1);
            s_data <<= 1;
            Pan_SSI_CK_H;
            delay_us(2);
            p_data <<= 1;
            if(Pan_SSI_DI)
            {
                p_data ++;
            }
            Pan_SSI_CK_L;
            delay_us(1);
        }
        Pan_SSI_CS_H;
    }
    OS_CRITICAL_EXIT(); //Õ∆≥Ÿ¡ŸΩÁ«¯
    return p_data;
}
u32 test_data[64] = {0};
u8 data_temp[6] = {0};
u8 state_temp[3] = {0};
u8 PT_Position_Record2[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
double temp_current = 0;
double RH_current = 0;
u32 cnt_temp = 0;
int Pan_A_Speed_Plus,Tilt_A_Speed_Plus;
int Speed_time_P,Speed_time_T;
int Init_P,Init_T;
int Scan_Motion_Mode;

double HotPursuit(double goal,double now,double step,int dir)
{
    if (goal == -1)
    {
        goal = (dir == 0) ? now + step : now - step;
    }
    else
    {
        if (dir)//1 ->
        {
					now = (now + step) > 360 ? ((now <= goal) ? now : now - 360) : now;
//            goal = (now + step) > goal ? goal : now + step;
					goal = (now > goal) ? now + step :((now + step) > goal ? goal : now + step);
//					goal = (goal > 360) ? goal - 360 : goal;
        }
        else//0 <-
        {
            now = (now - step) < 0 ? (now >= goal) ? now :now + 360 : now;
//            goal = (now - step) < goal ? goal : now - step;
					goal = (now < goal) ? now - step :((now - step) < goal ? goal : now - step);
//					goal = (goal < 0) ? goal + 360 : goal;
        }       
    }
    return goal;
}

int Up_Or_Down(double angle1,double angle2)//Âà§Êñ≠1ÊòØÂê¶Âú®2ÁöÑ‰∏äÈù¢ ÊòØËæìÂá∫1 Âê¶ËæìÂá∫0
{
    if(((angle1 - angle2) >= 0 && (angle1 - angle2) < 180) || ( (angle1 - angle2) <= 0 && (angle1 - angle2) <= -180))//1 ‘⁄ 2 …œ
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void Task_Timer(void *p)
{
    OS_ERR err;
//    u8 i;
//    char temp[] = {"temp:"};
//    char RH[] = {",RH:"};
//    char send_data[30] = {0};
//    char data_t[7] = {0};
//    char data_r[7] = {0};
//  u8 send_data[10] = {0};
//  u8 send_len = 0;
//  u8 USART_x = 0;
    int Speed_Record_Pan = 0;
    int Speed_Record_Tilt = 0;
    SHT30_IIC_Init();
    OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
    SHT30_WriteData(SHT30_ADDR,Period_Measurement_Command_Set);
    p = p;
#ifdef LASER_SETUP
    Laser_Init();//
#endif
    while(1)
    {
        //*****************************
        //Œππ∑
        IWDG_ReloadCounter();
//        //*****************************
//        //Œ¬ ™∂»≤…ºØ
//        cnt_temp ++;
//        if(cnt_temp == 100)
//        {
//            cnt_temp = 0;
//            SHT30_ReadData(SHT30_ADDR,Period_Measurement_Command_Read,data_temp,6);//∂¡Œ¬∂» ˝æ›
//
//            SHT30_ReadData(SHT30_ADDR,Status_Register_Command_Read,state_temp,3);//∂¡Œ¬∂»◊¥Ã¨ºƒ¥Ê∆˜
//            temp_current = (double)((data_temp[0] << 8) + data_temp[1]) * 175.0 / 65535.0 - 45.0;
//            RH_current = (double)((data_temp[3] << 8) + data_temp[4]) * 100.0 / 65535.0;
//
//            if(state_temp[0] != 0x80)
//            {
//                SHT30_WriteData(SHT30_ADDR,Period_Measurement_Command_Set);
//                temp_current = 0;
//            }
//            for(i = 0;i < 30;i ++)
//            {
//                send_data[i] = 0;
//            }
//
//            sprintf(data_t,"%3.3f",temp_current);
//            sprintf(data_r,"%3.3f",RH_current);
//            strcat(send_data,temp);
//            strcat(send_data,data_t);
//            strcat(send_data,RH);
//            strcat(send_data,data_r);
//            send_len = My_strlen(send_data);
//            USART_x = 2;
//            Post_Information_Function_TX(Sem_USART_TX,&USART_TX_BOX,USART_x,(u8 *)send_data,send_len);
//        }
        //*****************************
        //«Â≥˝ ˝æ›∂Ø◊˜
        if(Delete_Preset_FLag == 1 || Delete_Preset_FLag == 2)
        {
            Delete_Preset_Time ++;
            if(Delete_Preset_Time >= 300)
            {
                Delete_Preset_Time = 0;
                Delete_Preset_FLag = 0;
            }
        }
        if(Delete_Data_FLag == 1 || Delete_Data_FLag == 2)
        {
            Delete_Data_Time ++;
            if(Delete_Data_Time >= 300)
            {
                Delete_Data_Time = 0;
                Delete_Data_FLag = 0;
            }
        }
        //*****************************
        // µ ±ªÿ¥´
//      Return_Mode = Return_RealTime;
        Tim_Com ++;
        if(Tim_Com == 2)//20ms
        {
            if(Return_Mode == Return_RealTime)
            {
                Post_Information_Function(Sem_Analyze,&Analyze_BOX,Query_Angle_Tilt,0);
            }
        }
        if (Tim_Com >= 4)//40ms
        {
            if(Return_Mode == Return_RealTime)
            {
                Post_Information_Function(Sem_Analyze,&Analyze_BOX,Query_Angle_Pan,0);
            }
#ifdef LASER_SETUP
            laser_handle();
#endif
#ifdef ZS
            Post_Information_Function(Sem_Analyze,&Analyze_BOX,Return_Intime_Visca,0);
#endif
            Tim_Com = 0;
        }
        Tim_Com2 ++;
        if (Tim_Com2 == 50)//500ms
        {
            //Post_Information_Function(Sem_Analyze,&Analyze_BOX,Query_ZOOM,0);
            Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,CAM_ZoomPosInq,5);
						Tim_Com2 = 0;
        }
#ifdef ZS        
      if(Preset_Extract_Flag)//‘§÷√ŒªµΩŒª∑µªÿ
      {
          if(Place_InOut_Pan == InPlace && Place_InOut_Tilt == InPlace)
          {
              Preset_Extract_Flag = 0;
              Post_Information_Function(Sem_Analyze,&Analyze_BOX,Extract_Number_Require,0);
          }
      }
#endif
        //*****************************
        //Ãÿ ‚π¶ƒ‹
        switch(Special_Function)
        {
        case Tour_Enable:
        {
            if((Pan_Timer_Start == 1 && Tilt_Timer_Start == 1) || (Place_InOut_Pan == 1 && Place_InOut_Tilt == 1))
            {
                Stay_Time ++;
                if(Stay_Time >= Tour_Time)
                {
                    Stay_Time = 0;
                    Pan_Timer_Start = 0;
                    Tilt_Timer_Start = 0;
                    Post_Information_Function(Sem_Analyze,&Analyze_BOX,0xf000 + Preset_Extract,Preset_Num[Tour_Num]);
                    Tour_Num ++;
                    if(Tour_Num == Preset_Number)
                    {
                        Tour_Num = 0;
                    }
                }
            }
        }
        break;
        case Scan_Enable:
        {
            if (Section_Border2_Angle > ((Section_Border1_Angle > 180) ? Section_Border1_Angle - 180 : Section_Border1_Angle + 180))
            {
                Scan_Motion_Mode = Location_Motion_Scan;
            }
            else
            {
                Scan_Motion_Mode = Location_Motion;
            }
            switch(Scan_Num)
            {
            case 0://»•Section_Border2_Angle£¨µ±«∞‘⁄Section_Border1_Angle
            {
                if(fabs(Section_Border2_Angle - Pan_Real_Angle) > Pan_Angle_Accuracy)
                {
                    Place_InOut_Pan = OutPlace;
                    if (Scan_Motion_Mode == Location_Motion_Scan)
                    {
                        if(Pan_Target_Angle >= ((Section_Border2_Angle >=359.5)?Section_Border2_Angle - 360 : Section_Border2_Angle)&& Pan_Target_Angle < Section_Border2_Angle + 45)
                        {
                            Pan_Target_Angle = Section_Border2_Angle;
														//µ⁄“ª¥Œ‘À––∑¢ÀÕZOOM÷µ
														send_data[0] = 0x81;
														send_data[1] = 0x01;
														send_data[2] = 0x04;
														send_data[3] = 0x47;
														send_data[4] = Scan_Zoom_Left >> 12;
														send_data[5] = (Scan_Zoom_Left & 0x0f00) >> 8;
														send_data[6] = (Scan_Zoom_Left & 0x00f0) >> 4;
														send_data[7] = Scan_Zoom_Left & 0x000f;
														send_data[8] = 0xFF; 
														send_len = 9;
														Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);
                        }
                        else
                        {
                            Pan_Target_Angle = (Pan_Real_Angle > 315)?Pan_Real_Angle - 315:Pan_Real_Angle + 45;
                        }
                    }
                    else
                    {
                        Pan_Target_Angle = Section_Border2_Angle;
																									send_data[0] = 0x81;
														send_data[1] = 0x01;
														send_data[2] = 0x04;
														send_data[3] = 0x47;
														send_data[4] = Scan_Zoom_Left >> 12;
														send_data[5] = (Scan_Zoom_Left & 0x0f00) >> 8;
														send_data[6] = (Scan_Zoom_Left & 0x00f0) >> 4;
														send_data[7] = Scan_Zoom_Left & 0x000f;
														send_data[8] = 0xFF; 
														send_len = 9;
														Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);
                    }
                    Pan_Motion_Mode = Location_Motion;
                }
                else
                {
                    if (Stay_Time >= Scan_Time_Right)
                    {
                        Scan_Num = 1;
                        Stay_Time = 0;
                    }
                    else
                    {
                        Stay_Time ++;
                    }
                }
            }
            break;
            case 1://»•Section_Border1_Angle£¨µ±«∞‘⁄Section_Border2_Angle
            {
                if(fabs(Section_Border1_Angle - Pan_Real_Angle) > Pan_Angle_Accuracy)
                {
                    Place_InOut_Pan = OutPlace;
                    if (Scan_Motion_Mode == Location_Motion_Scan)
                    {
                        if(Pan_Target_Angle <= ((Section_Border1_Angle <= 0.05)?Section_Border1_Angle+360:Section_Border1_Angle ) && Pan_Target_Angle > Section_Border1_Angle - 45)
                        {
                            Pan_Target_Angle = Section_Border1_Angle;
														//µ⁄“ª¥Œ‘À––∑¢ÀÕZOOM÷µ
														send_data[0] = 0x81;
														send_data[1] = 0x01;
														send_data[2] = 0x04;
														send_data[3] = 0x47;
														send_data[4] = Scan_Zoom_Left >> 12;
														send_data[5] = (Scan_Zoom_Left & 0x0f00) >> 8;
														send_data[6] = (Scan_Zoom_Left & 0x00f0) >> 4;
														send_data[7] = Scan_Zoom_Left & 0x000f;
														send_data[8] = 0xFF; 
														send_len = 9;
												  	Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);
                        }
                        else
                        {
                            Pan_Target_Angle = (Pan_Real_Angle < 45)?Pan_Real_Angle + 315:Pan_Real_Angle - 45;
                        }
                    }
                    else
                    {
                        Pan_Target_Angle = Section_Border1_Angle;
														send_data[0] = 0x81;
														send_data[1] = 0x01;
														send_data[2] = 0x04;
														send_data[3] = 0x47;
														send_data[4] = Scan_Zoom_Left >> 12;
														send_data[5] = (Scan_Zoom_Left & 0x0f00) >> 8;
														send_data[6] = (Scan_Zoom_Left & 0x00f0) >> 4;
														send_data[7] = Scan_Zoom_Left & 0x000f;
														send_data[8] = 0xFF; 
														send_len = 9;
												  	Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);
                    }
                    Pan_Motion_Mode = Location_Motion;
                }
                else
                {
                    if (Stay_Time >= Scan_Time_Left)
                    {
                        Scan_Num = 0;
                        Stay_Time = 0;
                    }
                    else
                    {
                        Stay_Time ++;
                    }
                }
            }
            break;
            }
        }
        break;
        case Motor_Adaptation:                  //µÁª˙◊‘  ”¶≤‚ÀŸ
        {
            switch(Positioning_Sign)
            {
            case 0://◊º±∏Ω◊∂Œ
                Pan_Target_Angle = Pan_Real_Angle;
                Tilt_Target_Angle = Tilt_Limit_Down + 5;//¥π÷±»•œﬁŒªµ„∑¿÷ππ˝≥Â
                Pan_Motion_Mode = Location_Motion;
                Tilt_Motion_Mode = Location_Motion;
                Init_P = 0;
                Init_T = 0;
                if(Pan_Timer_Start == 1 && Tilt_Timer_Start == 1)
                {
                    Positioning_Sign ++;
                    Pan_A_MaxSpeed = 0;
                    Tilt_A_MaxSpeed = 0;
                    Pan_A_MinSpeed = 0;
                    Tilt_A_MinSpeed = 0;
                    Tilt_A_Speed_Plus = 0;
                    Pan_A_Speed_Plus = 0;//µΩŒª∫Û÷√¡„ ◊º±∏ø™ º≤‚ ‘
                    Scale_Velocity_Pan = 1.0;
                    Scale_Velocity_Tilt = 1.0;
                    Pan_Ultimate_MaxSpeed = 1000;
                    Tilt_Ultimate_MaxSpeed = 1000;
                }
                break;
            case 1://≤‚¡ø◊Ó¥ÛÀŸ∂»
                Pan_Motion_Mode = Rocker_Motion;
                Tilt_Motion_Mode = Rocker_Motion;
                Pan_Target_Speed = (Init_P) ? 0 : 10000;
                Tilt_Target_Speed = (Init_T) ? 0 : 10000;//∏¯“ª∏ˆÀŸ∂»º´¥Û÷µ »√pid»•Œﬁœﬁ¥Ûµƒ◊∑◊Ÿ’‚∏ˆÀŸ∂»£¨»Áπ˚ ±º‰µΩ¡ÀæÕ∏¯0»√∆‰Õ£œ¬¿¥
                /*≤‚¡ø∑Ω Ω£∫µ±PWMµΩ¥Ô◊Ó¥ÛPWM ±ø™ ºº∆ ± ’‚∂Œ ±º‰ƒ⁄»°µ√µƒ◊Ó¥ÛÀŸ∂»æÕ «◊Ó∏ﬂÀŸ*/
                if (PWM_P >= (Max_PWM - 10))
                {
                    Speed_Record_Pan = abs(pan_data.speed);
                    if (Speed_Record_Pan > Pan_A_MaxSpeed )
                    {
                        Pan_A_MaxSpeed = Speed_Record_Pan;
                        Pan_A_Speed_Plus = Pan_A_Speed;
                        //Speed_time_P = 0;
                    }
                    Speed_time_P++;
                }
                if (PWM_T >= (Max_PWM - 10))
                {
                    Speed_Record_Tilt = abs(tilt_data.speed);
                    if (Speed_Record_Tilt > Tilt_A_MaxSpeed)
                    {
                        Tilt_A_MaxSpeed = Speed_Record_Tilt;
                        Tilt_A_Speed_Plus = Tilt_A_Speed;
                        //Speed_time_T = 0;
                    }
                    Speed_time_T++;
                }
                if (Speed_time_P > 200)//µ•Œª£∫10ms
                {
                    Init_P = 1;// ±º‰µΩ¡ÀæÕπÿ±’ Õ£÷π‘À∂Ø
                }
                if (Speed_time_T > 200)
                {
                    Init_T = 1;
                }
                if (Init_P && Init_T)
                {
                    if(!Pan_A_Speed && !Tilt_A_Speed)
                    {
                        Positioning_Sign ++;//ÕÍ»´Õ£÷π≤≈ƒ‹Ω¯»Î◊Ó–°ÀŸ∂»≤‚¡ø ≤ª»ªª·∏…»≈ÀŸ∂»≤…ºØ
                    }
                }
                break;
            case 2://≤‚¡ø◊Ó–°ÀŸ∂»
                Pan_Target_Speed = (Pan_A_Speed > 2) ? 0 : Pan_Target_Speed + 1 ;
                Tilt_Target_Speed = (Tilt_A_Speed > 2) ? 0 : Tilt_Target_Speed + 1 ;
                if(Pan_A_Speed > 2)
                {
                    Pan_A_MinSpeed = Pan_A_Speed * Pan_A_MaxSpeed / Pan_A_Speed_Plus;
                }
                if(Tilt_A_Speed > 2)
                {
                    Tilt_A_MinSpeed = Tilt_A_Speed * Tilt_A_MaxSpeed / Tilt_A_Speed_Plus;
                }
                if (!Pan_Target_Speed && !Tilt_Target_Speed)
                {
                    Positioning_Sign = 0;
                    Speed_time_P = 0;
                    Speed_time_T = 0;
                    //º∆À„ÀŸ∂»±»¿˝œµ ˝
                    Scale_Velocity_Pan = (float)Pan_A_MaxSpeed / Pan_A_Speed_Plus;
                    Scale_Velocity_Tilt = (float)Tilt_A_MaxSpeed / Tilt_A_Speed_Plus;
                    Pan_Ultimate_MaxSpeed = Pan_A_MaxSpeed * 0.9;
                    Tilt_Ultimate_MaxSpeed = Tilt_A_MaxSpeed * 0.9;
                    Pan_Ultimate_MinSpeed = Pan_A_MinSpeed * 1.2;
                    Tilt_Ultimate_MinSpeed = Tilt_A_MinSpeed * 1.2;
                    //∑µªÿ¡„µ„
                    Pan_Motion_Mode = Location_Motion;
                    Tilt_Motion_Mode = Location_Motion;
                    Special_Function = Tour_Scan_Disable;
                    Pan_Target_Angle = 0;
                    Tilt_Target_Angle = 0;
                    //¥Ê ¥¢e∑Ω
                    PT_Position_Record2[1] = (Pan_Ultimate_MaxSpeed >> 8)  & 0xff;
                    PT_Position_Record2[2] = Pan_Ultimate_MaxSpeed & 0xff;
                    PT_Position_Record2[3] = (Tilt_Ultimate_MaxSpeed >> 8)  & 0xff;
                    PT_Position_Record2[4] = Tilt_Ultimate_MaxSpeed & 0xff;
                    PT_Position_Record2[5] = (Pan_Ultimate_MinSpeed >> 8)  & 0xff;
                    PT_Position_Record2[6] = Pan_Ultimate_MinSpeed & 0xff;
                    PT_Position_Record2[7] = (Tilt_Ultimate_MinSpeed >> 8)  & 0xff;
                    PT_Position_Record2[8] = Tilt_Ultimate_MinSpeed & 0xff;
                    PT_Position_Record2[9] = ((u16)(Scale_Velocity_Pan * 100) >> 8) & 0xff;
										PT_Position_Record2[10] = (u16)(Scale_Velocity_Pan * 100) & 0xff;
                    PT_Position_Record2[11] = ((u16)(Scale_Velocity_Tilt * 100) >> 8) & 0xff;
										PT_Position_Record2[12] = (u16)(Scale_Velocity_Tilt * 100) & 0xff;
                    PT_Position_Record2[13] = (u8)(Speed_Scale_Pan * 100);
                    PT_Position_Record2[14] = (u8)(Speed_Scale_Tilt * 100);
                    PT_Position_Record2[15] = Speed_Set_Satus;
                    EEPRom_WriteDataLen(EEPROM_Parameter,Max_Speed,PT_Position_Record2,Preset_Bytes_NUN);
                    //ªÿ∏¥√¸¡Ó
                    send_data[2] = 0xa6;
                    send_data[3] = 0x11;
                    send_data[4] = 0x00;
                    send_data[5] = 0xff;
                    send_data[6] = Check_Sum(send_data,1,6);
                    send_len = 7;
                    Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,1,send_data,send_len);
                    Post_Information_Function_TX(Sem_USART_TX,&USART2_TX_BOX,2,send_data,send_len);
                    Speed_Set_Satus = 1;//…Ë÷√ÕÍ≥…
                }
                break;
            default:
                break;
            }
        }
        break;
        case Rotating:
        {
            if(Rotating_Direction == 0)//À≥ ±’Î
            {
                Pan_Target_Speed = Speed_Rotate;
            }
            if(Rotating_Direction == 1)//ƒÊ ±’Î
            {
                Pan_Target_Speed = ~Speed_Rotate + 1;
            }
            Pan_Motion_Mode = Rocker_Motion;
        }
        break;
		case Skin_Fruit_ON://ÀÆπ˚∆§
		{
#ifdef SH
            if (fabs(Cruise.P_data_2 - Cruise.P_data_1) < FOV / 3)//¬›–˝…®ƒ£ Ω
            {
                switch(Cruise.situation)
                {
                    case 1://–˝◊™“ª»¶
                    {
											Cruise.flag -= (Cruise.flag > 0) ? 1 : 0;
											Pan_Target_Angle = (Cruise.flag > 0) ? HotPursuit(Cruise.P_data_2 + (Cruise.dir ? 1: -1) * 90,Pan_Real_Angle,Cruise.speed,!Cruise.dir) : HotPursuit(Cruise.P_data_2,Pan_Real_Angle,Cruise.speed,!Cruise.dir);
											Pan_Motion_Mode = Location_Motion;
                        Place_InOut_Pan = OutPlace;
                        Pan_Location_Speed = Cruise.speed_p;
                        if (Pan_Timer_Start == 1)
                        {
                            Cruise.T_data_temp = Tilt_Real_Angle;
                            Cruise.flag = Cruise.situation;
                            Pan_Motion_Mode = Rocker_Motion;
													if (Cruise.last)
													{
															Cruise.situation = 2;
													}
													else
													{
															Cruise.situation -= Cruise.situation;
													}
                        }
                    }break;
                    case 0://¥π÷±◊ﬂ“ª∏ˆÃ›∂»
                    {
//												Cruise.T_data_temp += (Cruise.T_data_temp - Cruise.grads) < 0 ? 360 : 0;
												if(Up_Or_Down(Cruise.T_data_1, Cruise.T_data_2))//1 ‘⁄ 2 …œ
                        {
                            //if (((Cruise.T_data_temp - Cruise.grads) < Cruise.T_data_2) && Cruise.T_data_temp > Cruise.T_data_2)
                            if (Up_Or_Down(Cruise.T_data_2,((Cruise.T_data_temp - Cruise.grads) < 0 ? (Cruise.T_data_temp - Cruise.grads) + 360 : (Cruise.T_data_temp - Cruise.grads))))
                            {
                                Tilt_Target_Angle = Cruise.T_data_2;
                            }
                            else
                            {
                                Tilt_Target_Angle = Cruise.T_data_temp - Cruise.grads;
                            }
														Tilt_Target_Angle = Tilt_Target_Angle < 0 ? Tilt_Target_Angle + 360 : Tilt_Target_Angle;
                            Tilt_Motion_Mode = Location_Motion;
                            Place_InOut_Tilt = OutPlace;
                        }
                        else
                        {
//														Cruise.T_data_temp -= (Cruise.T_data_temp + Cruise.grads) > 360 ? 360 : 0;
//														if (((Cruise.T_data_temp + Cruise.grads) > Cruise.T_data_2) && Cruise.T_data_temp < Cruise.T_data_2)
                            if (Up_Or_Down(((Cruise.T_data_temp + Cruise.grads) > 360 ? (Cruise.T_data_temp + Cruise.grads) - 360 : (Cruise.T_data_temp + Cruise.grads)),Cruise.T_data_2))
                            {
                                Tilt_Target_Angle = Cruise.T_data_2;
                            }
                            else
                            {
                                Tilt_Target_Angle = Cruise.T_data_temp + Cruise.grads;
                            }
                            Tilt_Target_Angle = Tilt_Target_Angle > 360 ? Tilt_Target_Angle - 360 : Tilt_Target_Angle;
														Tilt_Motion_Mode = Location_Motion;
														Place_InOut_Tilt = OutPlace;
                        }
                        Tilt_Location_Speed = Cruise.speed_t;
                        if(Tilt_Timer_Start == 1)
                        {
														if (Tilt_Target_Angle == Cruise.T_data_2)
														{
																Cruise.last = 1;
														}
														Cruise.situation += Cruise.flag;
                                                        Cruise.flag = Cruise.speed_p * 10 / Cruise.speed; 
                        }
                    }break;
                    case 2://≥ı º“‘º∞‘›Õ£÷ÿ∆Ù◊¥Ã¨
                    {
//												Pan_Target_Angle = (Cruise.status == go_on) ? Cruise.P_data_pause : Cruise.P_data_1;
//												Tilt_Target_Angle = (Cruise.status == go_on) ? Cruise.T_data_pause : Cruise.T_data_1;
                        Pan_Target_Angle = Cruise.P_data_1;
                        Tilt_Target_Angle = Cruise.T_data_1;
                        Pan_Motion_Mode = Location_Motion;
                        Place_InOut_Pan = OutPlace;
                        Tilt_Motion_Mode = Location_Motion;
                        Place_InOut_Tilt = OutPlace;
												Cruise.last = 0;
                        if (Pan_Timer_Start == 1 && Tilt_Timer_Start == 1)
                        {
                            Cruise.situation = 1;
														Cruise.flag = Cruise.speed_p * 10 / Cruise.speed;
														Cruise.T_data_temp = Tilt_Real_Angle;
														Tilt_Motion_Mode = Rocker_Motion;
														Pan_Motion_Mode = Rocker_Motion;
                        }
                    }break;
                    default:
                    {

                    }break;
									}										
            }
            else//≤¡≤£¡ßƒ£ Ω
            {
                switch(Cruise.situation)
                {
                    case 1://goto 2
                    {
                        Pan_Target_Angle = HotPursuit(Cruise.P_data_2,Pan_Real_Angle,Cruise.speed,!Cruise.dir);
												Pan_Motion_Mode = Location_Motion;
                        Place_InOut_Pan = OutPlace;
                        Pan_Location_Speed = Cruise.speed_p;
                        if (Pan_Timer_Start == 1)
                        {
                            Cruise.T_data_temp = Tilt_Real_Angle;
                            Cruise.flag = Cruise.situation;
													if (Cruise.last)
													{
															Cruise.situation = 2;
													}
													else
													{
															Cruise.situation -= Cruise.situation;
													}
                        }
                    }break;
                    case 0://¥π÷±◊ﬂ“ª∏ˆÃ›∂»
                    {
                        if(Up_Or_Down(Cruise.T_data_1, Cruise.T_data_2))//1 ‘⁄ 2 …œ
                        {
                            //if (((Cruise.T_data_temp - Cruise.grads) < Cruise.T_data_2) && Cruise.T_data_temp > Cruise.T_data_2)
                            if (Up_Or_Down(Cruise.T_data_2, ((Cruise.T_data_temp - Cruise.grads) < 0 ? (Cruise.T_data_temp - Cruise.grads) + 360 : (Cruise.T_data_temp - Cruise.grads))))
                            {
                                Tilt_Target_Angle = Cruise.T_data_2;
                            }
                            else
                            {
                                Tilt_Target_Angle = Cruise.T_data_temp - Cruise.grads;
                            }
														Tilt_Target_Angle = Tilt_Target_Angle < 0 ? Tilt_Target_Angle + 360 : Tilt_Target_Angle;
                            Tilt_Motion_Mode = Location_Motion;
                            Place_InOut_Tilt = OutPlace;
                        }
                        else
                        {
                            //if (((Cruise.T_data_temp + Cruise.grads) > Cruise.T_data_2) && Cruise.T_data_temp < Cruise.T_data_2)
                            if (Up_Or_Down(((Cruise.T_data_temp + Cruise.grads) > 360 ? (Cruise.T_data_temp + Cruise.grads) - 360 : (Cruise.T_data_temp + Cruise.grads)),Cruise.T_data_2))
                            {
                                Tilt_Target_Angle = Cruise.T_data_2;
                            }
                            else
                            {
                                Tilt_Target_Angle = Cruise.T_data_temp + Cruise.grads;
                            }
                            Tilt_Target_Angle = Tilt_Target_Angle > 360 ? Tilt_Target_Angle - 360 : Tilt_Target_Angle;
														Tilt_Motion_Mode = Location_Motion;
														Place_InOut_Tilt = OutPlace;
                        }
                        Tilt_Location_Speed = Cruise.speed_t;
                        if(Tilt_Timer_Start == 1)
                        {
														if (Tilt_Target_Angle == Cruise.T_data_2)
														{
																Cruise.last = 1;
														}
																Cruise.situation -= Cruise.flag;
                        }
                    }break;
                    case -1://goto 1
                    {
                        Pan_Target_Angle = HotPursuit(Cruise.P_data_1,Pan_Real_Angle,Cruise.speed,Cruise.dir);
												Pan_Motion_Mode = Location_Motion;
                       Place_InOut_Pan = OutPlace;
                       Pan_Location_Speed = Cruise.speed_p;
                        if (Pan_Timer_Start == 1)
                        {
                            Cruise.T_data_temp = Tilt_Real_Angle;
                            Cruise.flag = Cruise.situation;
													if (Cruise.last)
													{
															Cruise.situation = 2;
													}
													else
													{
															Cruise.situation -= Cruise.situation;
													}
                        }
                    }break;
                    case 2://≥ı º“‘º∞‘›Õ£÷ÿ∆Ù◊¥Ã¨
                    {
//												Pan_Target_Angle = (Cruise.status == go_on) ? Cruise.P_data_pause : Cruise.P_data_1;
//												Tilt_Target_Angle = (Cruise.status == go_on) ? Cruise.T_data_pause : Cruise.T_data_1;
                        Pan_Target_Angle = Cruise.P_data_1;
                        Tilt_Target_Angle = Cruise.T_data_1;
                        Pan_Motion_Mode = Location_Motion;
                        Place_InOut_Pan = OutPlace;
                        Tilt_Motion_Mode = Location_Motion;
                        Place_InOut_Tilt = OutPlace;
												Cruise.last = 0;
                        if (Pan_Timer_Start == 1 && Tilt_Timer_Start == 1)
                        {
                            Cruise.situation = 1;
														Cruise.flag = -1;
														Cruise.T_data_temp = Tilt_Real_Angle;
														Tilt_Motion_Mode = Rocker_Motion;
														Pan_Motion_Mode = Rocker_Motion;
                        }
                    }break;
                    default:
                    {

                    }break;
                }break;
            }
            


#endif
#ifdef HY
			switch(Fruit.Statt_Mode)
			{//◊º±∏◊¥Ã¨
				case 0:
				{
				//º«¬ºµ±«∞ÀÆ∆Ω÷µ
					Skin_PAN = Pan_Real_Angle;
					Fruit.Forerunner = Pan_Real_Angle;
					//»•…œœﬁŒªµ»¥˝
					Tilt_Target_Angle = Fruit.Up;//¥π÷±»•…œœﬁŒª◊º±∏
                    Tilt_Motion_Mode = Location_Motion;
					Fruit.Statt_Mode++;
				}break;
			//æÕ–˜◊¥Ã¨
				case 1:
				{
				//œ»µ»¥˝…œœﬁŒªæÕŒª
					if(fabs(Tilt_Target_Angle - Tilt_Real_Angle) > Pan_Angle_Accuracy * 5)
						break; 
						//»Áπ˚“—æ≠æÕŒª£¨‘Úø™ ºœ˜ÀÆπ˚
						Pan_Motion_Mode = Rocker_Motion;
                        Tilt_Motion_Mode = Rocker_Motion;
						if(Fruit.Direction == 1)//’˝∑ΩœÚ
						{
							Pan_Target_Speed = Fruit.Speed;
							Fruit.Forerunner+=12;
						}
						else if(Fruit.Direction == 0)//∑¥∑ΩœÚ
						{	
							Pan_Target_Speed = ~Fruit.Speed+1;
							Fruit.Forerunner-=12;
						}
						Tilt_Target_Speed = 0;
						Skin_Tilt = Tilt_Real_Angle;
						Fruit.Statt_Mode++;
				}break;
				//‘À––◊¥Ã¨
				case 2:
				{
					double err;
					if(Fruit.Direction == 1)//À≥ ±’Î‘À–– ±,œÚ”“
					{
					    if(Fruit.Forerunner>360)
						{
							Fruit.Forerunner-=360;								
						}
						else if((int)Fruit.Forerunner != (int)Skin_PAN )
						{
							Fruit.Forerunner += 1;
						}
								//“—æ≠◊∑…œƒø±Í£¨Ω¯––≈–∂œ
						else
						{
							if(fabs(Skin_PAN-Pan_Real_Angle) <= 7 ||(Pan_Real_Angle <=10 && (Skin_PAN-Pan_Real_Angle>345)))
							{
								if(Skin_Tilt != Fruit.Down)
								{
								//∏©—ˆΩ«∂»ø™ º±‰ªØ
									Tilt_Motion_Mode = Location_Motion;
									Tilt_Target_Angle = Skin_Tilt-Fruit.Tilt_Dif;
									if(Tilt_Target_Angle<0)
									{
										Tilt_Target_Angle+=360;
									}
									//œ»≈–∂œ «∑Ò‘⁄Õ¨“ª≤‡ 360Œ™ΩÁ°£
									if(Skin_Tilt-Fruit.Down > 0)
									{
										err = Skin_Tilt - Fruit.Down;
									}
									else
									{
										err = Skin_Tilt - Fruit.Down + 360;
									}
											if(err <=Fruit.Tilt_Dif)
											{
												Tilt_Target_Angle = Fruit.Down;
												Skin_Tilt =Fruit.Down;
											}
											else
											{
												Skin_Tilt = Tilt_Target_Angle;
											}
											Skin_PAN = Pan_Real_Angle;
											Pan_Target_Speed = Fruit.Speed;
											Fruit.Forerunner =Skin_PAN+12;
										}
										else//“—æ≠µΩœ¬œﬁŒªΩ· ¯¡À£¨∑µªÿ0µ„
										{
											Pan_Target_Speed = 0;
											Fruit.Statt_Mode = 0;
										}
									}
								}
							}
							else if(Fruit.Direction == 0)//À≥ ±’Î‘À–– ±£¨œÚ◊Û
							{
								if(Fruit.Forerunner < 0)
								{
									Fruit.Forerunner += 360;								
								}
								else if(Fruit.Forerunner != Skin_PAN )
								{
									Fruit.Forerunner -= 1;
								}
								//“—æ≠◊∑…œƒø±Í£¨Ω¯––≈–∂œ
								else
								{
									if(fabs(Skin_PAN-Pan_Real_Angle) <= 7 ||(Pan_Real_Angle <=10 && (Skin_PAN-Pan_Real_Angle>345)))
									{
										if(Skin_Tilt != Fruit.Down)
										{
											//∏©—ˆΩ«∂»ø™ º±‰ªØ
											Tilt_Motion_Mode = Location_Motion;
											Tilt_Target_Angle = Skin_Tilt-Fruit.Tilt_Dif;
											if(Tilt_Target_Angle<0)
											{
												Tilt_Target_Angle+=360;
											}
											//œ»≈–∂œ «∑Ò‘⁄Õ¨“ª≤‡ 360Œ™ΩÁ°£
											if(Skin_Tilt-Fruit.Down > 0)
											{
												err = Skin_Tilt - Fruit.Down;
											}
											else
											{
												err = Skin_Tilt - Fruit.Down + 360;
											}
											if(err <=Fruit.Tilt_Dif)
											{
												Tilt_Target_Angle = Fruit.Down;
												Skin_Tilt =Fruit.Down;
											}
											else
											{
												Skin_Tilt = Tilt_Target_Angle;
											}
											Skin_PAN = Pan_Real_Angle;
											Pan_Target_Speed = ~Fruit.Speed+1;
											Fruit.Forerunner =Skin_PAN - 12;
										}
										else//“—æ≠µΩœ¬œﬁŒªΩ· ¯¡À£¨∑µªÿ0µ„
										{
											Pan_Target_Speed = 0;
											Fruit.Statt_Mode = 0;
										}
									}
								}
							}
						}
					}
#endif
		}break;      
        case Tour_Scan_Disable:
        {
        }//break;
        default:
        {
            if( ( Watcher_Switch == ON ) && ( watcher_pause_state == watcher_continue ) )       //Â¶ÇÊûúÂÆàÊúõ‰ΩçÂ§Ñ‰∫éÂºÄÂêØÁä∂ÊÄÅÔºå‰∏îÂÆàÊúõ‰ΩçÊöÇÂÅúÁä∂ÊÄÅÊòØ‚ÄúÁªßÁª≠‚Ä?
            {
                if((PWM_P == 0 && PWM_T == 0) || (Pan_Timer_Start == 1 && Tilt_Timer_Start == 1))   //‰∫ëÂè∞Â§Ñ‰∫éÂÅúÊ≠¢Áä∂ÊÄÅÊàñÂ§Ñ‰∫éËåÉÂõ¥ÂÜ?
                {
                    Stay_Time ++;               //ÂÅúÁïôÊó∂Èó¥Ëá™Â¢û
                    if(Stay_Time >= Watcher_Time)   //Â¶ÇÊûúÂÅúÁïôÊó∂Èó¥Âà∞ËææÂÆàÊúõ‰ΩçËÆæÂÆöÊó∂Èó?
                    {
                        Pan_Location_Speed = Tour_Speed_Pan;
                        Tilt_Location_Speed = Tour_Speed_Tilt;
                        Pan_Target_Angle = Watcher_Angle_Pan;
                        Tilt_Target_Angle = Watcher_Angle_Tilt;
                        Pan_Motion_Mode = Location_Motion;
                        Tilt_Motion_Mode = Location_Motion;
                        Place_InOut_Pan = OutPlace;
                        Place_InOut_Tilt = OutPlace;
                    }
                }
                else
                {
                    Stay_Time = 0;
                }
            }
        }
        break;
        }
        OSTimeDlyHMSM(0,0,0,Task_Timer_Cycle_Time,OS_OPT_TIME_HMSM_STRICT,&err);
    }
}
