#include "sys.h"
#include "24LCxx.h"
#include "parameters.h"
#include "TaskAnalyze.h"
#include "SpecialFunctions.h"
#include "RockerControl.h"
#include "TaskEncoder.h"
#include "PID.h"
#include "TaskMotor.h"
u16 Location_Motion_Permisson(double Target_Angle_Old,double Target_Anlge_New,double resolution)//判断两点距离差值的函数
{
    if(fabs(Target_Angle_Old - Target_Anlge_New) < resolution)
    {
        return 1;
    }
    else
    {
        return 0;
    }       
}

u16 Usart1_Send_Boot(void)//启动发送指令
{
    u8 USART_x = 0;
    u8 send_len = 0;
    u8 send_data[7] = {0};
		for (int i = 0;i<3;i++)
		{
				USART_x = 3;
			switch(i)
			{
				case 0:
					send_data[0] = 0x81;
					send_data[1] = 0x02;
					send_data[2] = 0x09;
					send_data[3] = 0x00;
					send_data[4] = 0x03;
					send_data[5] = 0xFF;
					send_len = 6;break;
				case 1:
					send_data[0] = 0x81;
					send_data[1] = 0x02;
					send_data[2] = 0x09;
					send_data[3] = 0x00;
					send_data[4] = 0x04;
					send_data[5] = 0xFF;
					send_len = 6;break;
				case 2:
					send_data[0] = 0x81;
					send_data[1] = 0x02;
					send_data[2] = 0x0B;
					send_data[3] = 0x00;
					send_data[4] = 0x06;
					send_data[5] = 0xFF;
					send_len = 6;break;
				default:break;
			}
				Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);
				delay_ms(50);
		}
		return 0;		
}

void Tracking_Fuction_Recording(u16 CMD,u16 Data)//录制
{
    if (CMD)//不是停止命令
    {
        Tracking[StartML].Direction = CMD;
        Tracking[StartML].Speed = Data;
        Tracking[StartML].Start.Pan = Pan_Real_Angle;
        Tracking[StartML].Start.Tilt = Tilt_Real_Angle;
        zoom_record = 0;
    }
    else//停止命令
    {
        if(Location_Motion_Permisson(Tracking[StartML - 1].Stop.Pan,Pan_Real_Angle,0.05) && Location_Motion_Permisson(Tracking[StartML - 1].Stop.Tilt,Tilt_Real_Angle,0.05))
        {
            return;
        }
        else
        {
            Tracking[StartML].Stop.Pan = Pan_Real_Angle;
            Tracking[StartML].Stop.Tilt = Tilt_Real_Angle;
            Tracking[StartML].Zoom = Zoom_Position;
            StartML ++;
        }
    }
}
int Scan_Motion_Mode;
u8 PT_Position_Record2[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int Pan_A_Speed_Plus,Tilt_A_Speed_Plus;
int Speed_time_P,Speed_time_T;
int Init_P,Init_T;
int Speed_Record_Pan = 0;
int Speed_Record_Tilt = 0;
u8 send_len = 0;
u8 send_data[20] = {0xff,0x01};
u8 USART_x = 3;
u8 EEPROM_senddata[17] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int manual_counter = 0;
#ifdef HY
u8 Cmd_Last=0;
int Tracking_Counter = 0;
u8 Tracking_State = 0;
#endif
#ifdef HY
double Skin_PAN;
double Skin_Tilt;
#endif
void SpecialFunctions(void)//特殊功能
{
        switch(Special_Function)
        {
        case Tour_Enable://巡检
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
        case Scan_Enable://传统扫描
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
                    //存储e方
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
        case Rotating://旋转
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
        case Skin_Fruit_Pause://苹果皮暂停
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
#ifdef HY
				case Tracking_ON://学习路径复现
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
				case Auto_Tracking:
				{
						manual_counter += (manual_counter > 65535)?0:1;
						if(manual_counter > 500)
						{
								Pan_Motion_Mode = Rocker_Motion;
								Tilt_Motion_Mode = Rocker_Motion;																	//锁定电机模式为摇杆模式
							  Pan_Target_Speed = 0;                                               //锁定速度为0，跳过摇杆和位置模式进入第三模式控制
								Tilt_Target_Speed = 0;
								AutoTrackingMotor(Tracking_Speed_Pan,pan_data.speed,&Pan_ppid,0);
								AutoTrackingMotor(Tracking_Speed_Tilt,tilt_data.speed,&Tilt_ppid,1);
						}	
				}break;
        case Tour_Scan_Disable://无操作，原始状态
        {
						if(rocker_mark)//手控介入
								manual_counter = 0;//计时器清零
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
}
