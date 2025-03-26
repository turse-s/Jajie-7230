#include "TaskStart.h"
#include "TaskTimer.h"
#include "TaskUSART_TX.h"
#include "SHT30.h"
#include "TaskCAN.h"
#include "hardware.h"
#include "includes.h"
#include "delay.h"
#include "laser.h"
#include "TaskEncoder.h"
#include "TaskMotor.h"
#include "24LCxx.h"
#include "stmflash.h"
#include "TaskAnalyze.h"
u16 Tim_Com = 0;
u16 Tim_Com2 = 0;
u16 Tim_ML = 0;
u8 send_len = 0;
u8 send_data[20] = {0xff,0x01};
u8 USART_x = 3;
u8 EEPROM_senddata[17] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#ifdef HY
u8 Cmd_Last=0;
int Tracking_Counter = 0;
u8 Tracking_State = 0;
#endif
u32 Encoder_SPI_CMD(u16 cmd,u32 data)
{
    CPU_SR_ALLOC();
    u8 i;
    u32 p_data = 0;
    u32 s_data = 0;
    OS_CRITICAL_ENTER();//进入临界区
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
    OS_CRITICAL_EXIT(); //推迟临界区
    return p_data;
}
u8 PT_Position_Record2[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int Pan_A_Speed_Plus,Tilt_A_Speed_Plus;
int Speed_time_P,Speed_time_T;
int Init_P,Init_T;
int Scan_Motion_Mode;
#ifdef HY
double Skin_PAN;
double Skin_Tilt;
#endif
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
        //喂狗
        IWDG_ReloadCounter();
//        //*****************************
//        //温湿度采集
//        cnt_temp ++;
//        if(cnt_temp == 100)
//        {
//            cnt_temp = 0;
//            SHT30_ReadData(SHT30_ADDR,Period_Measurement_Command_Read,data_temp,6);//读温度数据
//
//            SHT30_ReadData(SHT30_ADDR,Status_Register_Command_Read,state_temp,3);//读温度状态寄存器
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
        //清除数据动作
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
        //实时回传
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
#ifdef RV
	#ifdef HY
						if(Swtich_va)
						{
								Post_Information_Function(Sem_Analyze,&Analyze_BOX,Return_Intime_Visca,0);
						}
	#else
								Post_Information_Function(Sem_Analyze,&Analyze_BOX,Return_Intime_Visca,0);
	#endif
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
      if(Preset_Extract_Flag)//预置位到位返回
      {
          if(Place_InOut_Pan == InPlace && Place_InOut_Tilt == InPlace)
          {
              Preset_Extract_Flag = 0;
              Post_Information_Function(Sem_Analyze,&Analyze_BOX,Extract_Number_Require,0);
          }
      }
#endif
#ifdef HY
        if (StartML > 0)
        {
            if (StartML < 25)
            {
                Tim_ML ++;
                if (Tim_ML >= 10000)//100s
                {
                    Post_Information_Function(Sem_Analyze,&Analyze_BOX,Direction_Stop,0);
                    Tracking[StartML + 1].Stop.Pan = Pan_Real_Angle;
                    Tracking[StartML + 1].Stop.Tilt = Pan_Real_Angle;
                    Tracking[StartML + 1].Zoom = Zoom_Position;
                    StartML = 0;
                    Tim_ML = 0;
                }
            }
            else
            {
                StartML = -1;
            }
        }
        else if (StartML == -1)
        {
            Tim_ML = 0;
        }				
#endif
        //*****************************
        //特殊功能
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
            case 0://去Section_Border2_Angle，当前在Section_Border1_Angle
            {
                if(fabs(Section_Border2_Angle - Pan_Real_Angle) > Pan_Angle_Accuracy)
                {
                    Place_InOut_Pan = OutPlace;
                    if (Scan_Motion_Mode == Location_Motion_Scan)
                    {
                        if(Pan_Target_Angle >= ((Section_Border2_Angle >=359.5)?Section_Border2_Angle - 360 : Section_Border2_Angle)&& Pan_Target_Angle < Section_Border2_Angle + 45)
                        {
                            Pan_Target_Angle = Section_Border2_Angle;
                        }
                        else
                        {
                            Pan_Target_Angle = (Pan_Real_Angle > 315)?Pan_Real_Angle - 315:Pan_Real_Angle + 45;
                        }
                    }
                    else
                    {
                        Pan_Target_Angle = Section_Border2_Angle;
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
            case 1://去Section_Border1_Angle，当前在Section_Border2_Angle
            {
                if(fabs(Section_Border1_Angle - Pan_Real_Angle) > Pan_Angle_Accuracy)
                {
                    Place_InOut_Pan = OutPlace;
                    if (Scan_Motion_Mode == Location_Motion_Scan)
                    {
                        if(Pan_Target_Angle <= ((Section_Border1_Angle <= 0.05)?Section_Border1_Angle+360:Section_Border1_Angle ) && Pan_Target_Angle > Section_Border1_Angle - 45)
                        {
                            Pan_Target_Angle = Section_Border1_Angle;
                        }
                        else
                        {
                            Pan_Target_Angle = (Pan_Real_Angle < 45)?Pan_Real_Angle + 315:Pan_Real_Angle - 45;
                        }
                    }
                    else
                    {
                        Pan_Target_Angle = Section_Border1_Angle;
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
						case 2:
						{
									if( abs(Zoom_Position - Scan_Zoom_Left) < 50)
									{
											Scan_Num = 1;
									}
						}break;
            }
        }
        break;
        case Motor_Adaptation:                  //电机自适应测速
        {
            switch(Positioning_Sign)
            {
            case 0://准备阶段
                Pan_Target_Angle = Pan_Real_Angle;
                Tilt_Target_Angle = Tilt_Limit_Down + 5;//垂直去限位点防止过冲
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
                    Pan_A_Speed_Plus = 0;//到位后置零 准备开始测试
                    Scale_Velocity_Pan = 1.0;
                    Scale_Velocity_Tilt = 1.0;
                    Pan_Ultimate_MaxSpeed = 1000;
                    Tilt_Ultimate_MaxSpeed = 1000;
                }
                break;
            case 1://测量最大速度
                Pan_Motion_Mode = Rocker_Motion;
                Tilt_Motion_Mode = Rocker_Motion;
                Pan_Target_Speed = (Init_P) ? 0 : 10000;
                Tilt_Target_Speed = (Init_T) ? 0 : 10000;//给一个速度极大值 让pid去无限大的追踪这个速度，如果时间到了就给0让其停下来
                /*测量方式：当PWM到达最大PWM时开始计时 这段时间内取得的最大速度就是最高速*/
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
                if (Speed_time_P > 200)//单位：10ms
                {
                    Init_P = 1;//时间到了就关闭 停止运动
                }
                if (Speed_time_T > 200)
                {
                    Init_T = 1;
                }
                if (Init_P && Init_T)
                {
                    if(!Pan_A_Speed && !Tilt_A_Speed)
                    {
                        Positioning_Sign ++;//完全停止才能进入最小速度测量 不然会干扰速度采集
                    }
                }
                break;
            case 2://测量最小速度
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
                    //计算速度比例系数
                    Scale_Velocity_Pan = (float)Pan_A_MaxSpeed / Pan_A_Speed_Plus;
                    Scale_Velocity_Tilt = (float)Tilt_A_MaxSpeed / Tilt_A_Speed_Plus;
                    Pan_Ultimate_MaxSpeed = Pan_A_MaxSpeed * 0.9;
                    Tilt_Ultimate_MaxSpeed = Tilt_A_MaxSpeed * 0.9;
                    Pan_Ultimate_MinSpeed = Pan_A_MinSpeed * 1.2;
                    Tilt_Ultimate_MinSpeed = Tilt_A_MinSpeed * 1.2;
                    //返回零点
                    Pan_Motion_Mode = Location_Motion;
                    Tilt_Motion_Mode = Location_Motion;
                    Special_Function = Tour_Scan_Disable;
                    Pan_Target_Angle = 0;
                    Tilt_Target_Angle = 0;
                    //存 储e方
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
                    //回复命令
                    send_data[2] = 0xa6;
                    send_data[3] = 0x11;
                    send_data[4] = 0x00;
                    send_data[5] = 0xff;
                    send_data[6] = Check_Sum(send_data,1,6);
                    send_len = 7;
                    Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,1,send_data,send_len);
                    Post_Information_Function_TX(Sem_USART_TX,&USART2_TX_BOX,2,send_data,send_len);
                    Speed_Set_Satus = 1;//设置完成
                }
                break;
            default:
                break;
            }
        }
        break;
        case Rotating:
        {
            if(Rotating_Direction == 0)//顺时针
            {
                Pan_Target_Speed = Speed_Rotate;
            }
            if(Rotating_Direction == 1)//逆时针
            {
                Pan_Target_Speed = ~Speed_Rotate + 1;
            }
            Pan_Motion_Mode = Rocker_Motion;
        }
        break;
#ifdef SH
        case Skin_Fruit_Pause:
        {
            Pan_Target_Angle = Cruise.P_data_pause;
			Tilt_Target_Angle = Cruise.T_data_pause;
            Pan_Motion_Mode = Location_Motion;                                    
			Tilt_Motion_Mode = Location_Motion;  
            if ((Place_InOut_Pan == InPlace && Place_InOut_Tilt == InPlace) && Cruise.status == recover)
            {
                Special_Function = Skin_Fruit_ON;
								Pan_Motion_Mode = Rocker_Motion;
								Tilt_Motion_Mode = Rocker_Motion;
                Cruise.status = start; 
            }
        }break;
#endif
		case Skin_Fruit_ON://水果皮
		{
#ifdef SH
            if (fabs(Cruise.P_data_2 - Cruise.P_data_1) < FOV / 3)//螺旋扫模式
            {
                switch(Cruise.situation)
                {
                    case 1://旋转一圈
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
                    case 0://垂直走一个梯度
                    {
//						Cruise.T_data_temp += (Cruise.T_data_temp - Cruise.grads) < 0 ? 360 : 0;
						Pan_Motion_Mode = Rocker_Motion;
						if(Up_Or_Down(Cruise.T_data_1, Cruise.T_data_2))//1 在 2 上
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
//							Cruise.T_data_temp -= (Cruise.T_data_temp + Cruise.grads) > 360 ? 360 : 0;
//							if (((Cruise.T_data_temp + Cruise.grads) > Cruise.T_data_2) && Cruise.T_data_temp < Cruise.T_data_2)
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
                    case 2://初始以及暂停重启状态
                    {
//						Pan_Target_Angle = (Cruise.status == go_on) ? Cruise.P_data_pause : Cruise.P_data_1;
//						Tilt_Target_Angle = (Cruise.status == go_on) ? Cruise.T_data_pause : Cruise.T_data_1;
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
            else//擦玻璃模式
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
                    case 0://垂直走一个梯度
                    {
                        Pan_Motion_Mode = Rocker_Motion;
						if(Up_Or_Down(Cruise.T_data_1, Cruise.T_data_2))//1 在 2 上
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
                    case 2://初始以及暂停重启状态
                    {
//						Pan_Target_Angle = (Cruise.status == go_on) ? Cruise.P_data_pause : Cruise.P_data_1;
//						Tilt_Target_Angle = (Cruise.status == go_on) ? Cruise.T_data_pause : Cruise.T_data_1;
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
			{//准备状态
				case 0:
				{
				//记录当前水平值
					Skin_PAN = Pan_Real_Angle;
					Fruit.Forerunner = Pan_Real_Angle;
					//去上限位等待
					Tilt_Target_Angle = Fruit.Up;//垂直去上限位准备
					Tilt_Timer_Start = 0;
					Tilt_Motion_Mode = Location_Motion;
					Fruit.Statt_Mode++;
				}break;
			//就绪状态
				case 1:
				{
				//先等待上限位就位
					if(Tilt_Timer_Start == 0)
						break; 
						//如果已经就位，则开始削水果
						Pan_Motion_Mode = Rocker_Motion;
                        Tilt_Motion_Mode = Rocker_Motion;
						if(Fruit.Direction == 1)//正方向
						{
							Pan_Target_Speed = Fruit.Speed;
							Fruit.Forerunner += 12;
						}
						else if(Fruit.Direction == 0)//反方向
						{	
							Pan_Target_Speed = ~Fruit.Speed+1;
							Fruit.Forerunner -= 12;
						}
						Tilt_Target_Speed = 0;
						Skin_Tilt = Tilt_Real_Angle;
						Fruit.Statt_Mode++;
				}break;
				//运行状态
				case 2:
				{
					double err;
					if(Fruit.Direction == 1)//顺时针运行时,向右
					{
					    if(Fruit.Forerunner>360)
						{
							Fruit.Forerunner-=360;								
						}
						else if(fabs(Fruit.Forerunner - Skin_PAN) >= 5)
						{
							Fruit.Forerunner  = Pan_Real_Angle + 2 * Fruit.Speed;
						}
								//已经追上目标，进行判断
						else
						{
							if(fabs(Skin_PAN-Pan_Real_Angle) <= 7 ||(Pan_Real_Angle <=10 && (Skin_PAN-Pan_Real_Angle>345)))
							{
								if(Skin_Tilt != Fruit.Down)
								{
								//俯仰角度开始变化
									Tilt_Motion_Mode = Location_Motion;
									Tilt_Target_Angle = Skin_Tilt-Fruit.Tilt_Dif;
									if(Tilt_Target_Angle<0)
									{
										Tilt_Target_Angle+=360;
									}
									//先判断是否在同一侧 360为界。
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
										else//已经到下限位结束了，返回0点
										{
											Pan_Target_Speed = 0;
											Fruit.Statt_Mode = 0;
										}
									}
								}
							}
							else if(Fruit.Direction == 0)//顺时针运行时，向左
							{
								if(Fruit.Forerunner < 0)
								{
									Fruit.Forerunner += 360;								
								}
								else if(fabs(Fruit.Forerunner - Skin_PAN) >= 5)
								{
									Fruit.Forerunner = Pan_Real_Angle - 2 * Fruit.Speed;
								}
								//已经追上目标，进行判断
								else
								{
									if(fabs(Skin_PAN-Pan_Real_Angle) <= 7 ||(Pan_Real_Angle <=10 && (Skin_PAN-Pan_Real_Angle>345)))
									{
										if(Skin_Tilt != Fruit.Down)
										{
											//俯仰角度开始变化
											Tilt_Motion_Mode = Location_Motion;
											Tilt_Target_Angle = Skin_Tilt-Fruit.Tilt_Dif;
											if(Tilt_Target_Angle<0)
											{
												Tilt_Target_Angle+=360;
											}
											//先判断是否在同一侧 360为界。
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
										else//已经到下限位结束了，返回0点
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
        case EEPROM_CHECK:
        {
            if (EEPROM_checknum1 <= 50 && EEPROM_checknum1 >= 0)
            {
                EEPRom_ReadDataLen(EEPROM_Preset,EEPROM_checknum1,EEPROM_senddata,Preset_Bytes_NUN);
                Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,1,EEPROM_senddata,Preset_Bytes_NUN);
							  //USART1_Use_DMA_Tx_Flag = USART_DMA_Send_Data(USART1_DMA_Steam_TX,DMA_USART1_TX_Buf,USART1_TX_BOX.Memory_Data_1[USART1_TX_BOX.extract_finger],USART1_TX_BOX.Memory_Data[USART1_TX_BOX.extract_finger]);
								if (EEPROM_checknum1 >= 40)
								{
										delay_us(1000);
								}
                EEPROM_checknum1 ++;
            }
            else
            {
                EEPROM_checknum1 = -1;
								EEPROM_checknum0 = -1;
            }
//            if (EEPROM_checknum0 < EE_TYPE / Preset_Bytes_NUN && EEPROM_checknum1== -1)
//            {
//                EEPRom_ReadDataLen(EEPROM_Parameter,EEPROM_checknum0,EEPROM_senddata,Preset_Bytes_NUN);
//                Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,1,EEPROM_senddata,Preset_Bytes_NUN);
//                EEPROM_checknum0 ++;                
//            }
//            else
//            {
//                EEPROM_checknum0 = -1;
//            }
            if (EEPROM_checknum0 == -1 && EEPROM_checknum1 == -1)
            {
                Special_Function = Tour_Scan_Disable;
            }
        }break;
#ifdef HY
				 case Tracking_ON:
        {
            switch(Tracking_State)
            {
                case 0://goto起始点状态
                {
                    Pan_Target_Angle = Tracking[Tracking_Counter].Start.Pan;
                    Pan_Motion_Mode = Location_Motion;
                    Place_InOut_Pan = OutPlace;
                    Tilt_Target_Angle = Tracking[Tracking_Counter].Start.Tilt;
                    Tilt_Motion_Mode = Location_Motion;
                    Place_InOut_Tilt = OutPlace;
										
                    if (Location_Motion_Permisson(Tracking[Tracking_Counter].Start.Pan,Pan_Real_Angle,0.1) &&
                        Location_Motion_Permisson(Tracking[Tracking_Counter].Start.Tilt,Tilt_Real_Angle,0.1))
                    { 

						Tracking_State ++;
                    }

                }break;
                case 1://起始点就位 执行摇杆操作
                {
                    if(Tracking[Tracking_Counter].Direction != ZOOM_Tele || Tracking[Tracking_Counter].Direction != ZOOM_Wide)
                    {
                        Rocker_Control(Tracking[Tracking_Counter].Direction,Tracking[Tracking_Counter].Speed);
                        if(Location_Motion_Permisson(Tracking[Tracking_Counter].Stop.Pan,Pan_Real_Angle,(double)(Tracking[Tracking_Counter].Speed >> 8) / 50.0 + 0.1) &&
                           Location_Motion_Permisson(Tracking[Tracking_Counter].Stop.Tilt,Tilt_Real_Angle,(double)(Tracking[Tracking_Counter].Speed & 0xff) / 25.0 + 0.1))
                        {
                            Tracking_State ++;
													  send_data[0] = 0x81;
														send_data[1] = 0x01;
														send_data[2] = 0x04;
														send_data[3] = 0x47;
														send_data[4] = Tracking[Tracking_Counter].Zoom >> 12;
														send_data[5] = (Tracking[Tracking_Counter].Zoom & 0x0f00) >> 8;
														send_data[6] = (Tracking[Tracking_Counter].Zoom & 0x00f0) >> 4;
														send_data[7] = Tracking[Tracking_Counter].Zoom & 0x000f;
														send_data[8] = 0xFF; 
														send_len = 9;
														Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);
                        }
                    }
                    else
                    {
                        Tracking_State ++;
											  send_data[0] = 0x81;
                        send_data[1] = 0x01;
                        send_data[2] = 0x04;
                        send_data[3] = 0x47;
                        send_data[4] = Tracking[Tracking_Counter].Zoom >> 12;
                        send_data[5] = (Tracking[Tracking_Counter].Zoom & 0x0f00) >> 8;
                        send_data[6] = (Tracking[Tracking_Counter].Zoom & 0x00f0) >> 4;
                        send_data[7] = Tracking[Tracking_Counter].Zoom & 0x000f;
                        send_data[8] = 0xFF; 
                        send_len = 9;
                        Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);
                    }
                }break;
                case 2://临近停止区间 执行停止操作
                {                    
                    Rocker_Control(Direction_Stop,0);
					if(PWM_P == 0 && PWM_T ==0)
					{
                        if(abs((int)Tracking[Tracking_Counter].Zoom - (int)Zoom_Position) < 200)
                        {
                            Tracking_State ++ ;
                        }
					}
                }break;
                case 50://500ms后
                {
                    Tracking_Counter ++;
                    if(Tracking_Counter >= Tracking[0].counter)
                    {
                        Tracking_Counter = 0;
                    }
                    Tracking_State = 0;
                }break;
								default: Tracking_State ++;break;
            }   
        }break;
#endif
        case Tour_Scan_Disable:
        {
        }//break;
        default:
        {
            if( ( Watcher_Switch == ON ) && ( watcher_pause_state == watcher_continue ) )       //濡傛灉瀹堟湜浣嶅浜庡紑鍚姸鎬侊紝涓斿畧鏈涗綅鏆傚仠鐘舵�佹槸鈥滅户缁�?
            {
                if((PWM_P == 0 && PWM_T == 0) || (Pan_Timer_Start == 1 && Tilt_Timer_Start == 1))   //浜戝彴澶勪簬鍋滄鐘舵�佹垨澶勪簬鑼冨洿鍐?
                {
                    Stay_Time ++;               //鍋滅暀鏃堕棿鑷
                    if(Stay_Time >= Watcher_Time)   //濡傛灉鍋滅暀鏃堕棿鍒拌揪瀹堟湜浣嶈瀹氭椂闂?
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

void Rocker_Control(u16 Down_CMD,u16 Down_Data)
{
    switch(Down_CMD)
    {
			//*******************************************************************************************************************
			//*******************************************************************************************************************
																											//基本操作
			//*******************************************************************************************************************
			//*******************************************************************************************************************
    case Direction_Stop:  
        Pan_Motion_Mode = Rocker_Motion;                                    //设置水平摇杆模式
        Tilt_Motion_Mode = Rocker_Motion;                                   //设置垂直摇杆模式
        Pan_Target_Speed = 0;                                               //水平目标速度初始化
        Tilt_Target_Speed = 0;                                              //垂直目标速度初始化
        Stay_Time = 0;
        break;
    case Direction_Up:
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
				}
}
