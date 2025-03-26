#include "sys.h"
#include "ShiHui.h"
#include "TaskTimer.h"
#include "24LCxx.h"
#include "RockerControl.h"
#include "TaskUSART_TX.h"
#ifdef SH
u8 SH_Position_Record[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 SH_Position_Extract[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void ShiHuiFunctions(u16 Down_CMD,u16 Down_Data)
{
    	u8 send_len = 0;
			u8 send_data[20] = {0};
			u32 data_p = 0,data_t = 0;
		switch(Down_CMD)
		{
		case Left_Stay_Time:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if (scan_id > 0 && scan_id < 6)
        {
						Sweep[scan_id].time_left = Down_Data >> 8;            
            SH_Position_Record[1] = Sweep[scan_id].time_left;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),SH_Position_Record,Preset_Bytes_NUN);
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
            SH_Position_Record[2] = Sweep[scan_id].time_right;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),SH_Position_Record,Preset_Bytes_NUN);
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
            SH_Position_Record[3] = Sweep[scan_id].speed;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),SH_Position_Record,Preset_Bytes_NUN);
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
            SH_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
            SH_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
            SH_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
            SH_Position_Record[10] = Zoom_Position >> 8;
            SH_Position_Record[11] = Zoom_Position & 0xff;																		
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),SH_Position_Record,Preset_Bytes_NUN);
        }
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
            SH_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
            SH_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
            SH_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
            SH_Position_Record[12] =  Zoom_Position >> 8;
            SH_Position_Record[13] = Zoom_Position & 0xff;
            EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),SH_Position_Record,Preset_Bytes_NUN);
        }
    }
    break;
    case Start_Scan:
    {
        u16 scan_id;
        scan_id = (Down_Data & 0xff);
        if(EEPRom_ReadOneByte(EEPROM_Parameter,(scan_id + 230 - 1) * Preset_Bytes_NUN) == 0xaa)
        {
            EEPRom_ReadDataLen(EEPROM_Parameter,(scan_id + 230),SH_Position_Extract,Preset_Bytes_NUN);
            Sweep[scan_id].border_right = (double)(((SH_Position_Extract[7] | 0x0000) << 16) + ((SH_Position_Extract[8] | 0x0000) << 8) + SH_Position_Extract[9]) / 1000.0;
            Sweep[scan_id].border_left = (double)(((SH_Position_Extract[4] | 0x0000) << 16) + ((SH_Position_Extract[5] | 0x0000) << 8) + SH_Position_Extract[6]) / 1000.0;
            Sweep[scan_id].speed = SH_Position_Extract[3];
            Sweep[scan_id].time_left = SH_Position_Extract[1];
            Sweep[scan_id].time_right = SH_Position_Extract[2];
						Sweep[scan_id].Zoom_Position_Left = (SH_Position_Extract[10]  << 8) + SH_Position_Extract[11];
						Sweep[scan_id].Zoom_Position_Right = (SH_Position_Extract[12] << 8) + SH_Position_Extract[13];
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
        SH_Position_Record[1] = Sweep[scan_id].time_left;
        SH_Position_Record[2] = Sweep[scan_id].time_right;
        SH_Position_Record[3] = Sweep[scan_id].speed;
        SH_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
        SH_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
        SH_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
        SH_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
        SH_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
        SH_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
			  SH_Position_Record[12] =  Zoom_Position >> 8;
        SH_Position_Record[13] = Zoom_Position & 0xff;
				SH_Position_Record[10] = Zoom_Position >> 8;
        SH_Position_Record[11] = Zoom_Position & 0xff;	
        EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),SH_Position_Record,Preset_Bytes_NUN);
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
                EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,SH_Position_Extract,Preset_Bytes_NUN);
                data_p = 0;
                data_p |= SH_Position_Extract[10];
                data_p <<= 8;
                data_p |= SH_Position_Extract[11];
                data_p <<= 8;
                data_p |= SH_Position_Extract[12];
                data_t = 0;
                data_t |= SH_Position_Extract[13];
                data_t <<= 8;
                data_t |= SH_Position_Extract[14];
                data_t <<= 8;
                data_t |= SH_Position_Extract[15];
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
                EEPRom_ReadDataLen(EEPROM_Preset,Down_Data,SH_Position_Extract,Preset_Bytes_NUN);
                data_p = 0;
                data_p |= SH_Position_Extract[10];
                data_p <<= 8;
                data_p |= SH_Position_Extract[11];
                data_p <<= 8;
                data_p |= SH_Position_Extract[12];
                data_t = 0;
                data_t |= SH_Position_Extract[13];
                data_t <<= 8;
                data_t |= SH_Position_Extract[14];
                data_t <<= 8;
                data_t |= SH_Position_Extract[15];
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
                            Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);		
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
		default:break;		
}
}
#endif

