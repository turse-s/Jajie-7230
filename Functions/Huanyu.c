#include "sys.h"
#include "HuanYu.h"
#include "TaskTimer.h"
#include "24LCxx.h"
#include "RockerControl.h"
#ifdef HY
u8 HY_Position_Record[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 HY_Position_Extract[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void HuanYuFunctions(u16 Down_CMD,u16 Down_Data)
{
      u8 send_len = 0;
			u8 send_data[20] = {0};
			u16 i = 0,j = 0;
			switch(Down_CMD)
      {
          case Scan_Border_Left:
                {      
                      u16 scan_id;
                      scan_id = (Down_Data & 0xff);
                      if (scan_id < 6)
                      {
                          Sweep[scan_id].Zoom_Position_Left = Zoom_Position;
                          Sweep[scan_id].border_left = Pan_Real_Angle;
                          HY_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
                          HY_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
                          HY_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
                          HY_Position_Record[10] = Zoom_Position >> 8;
                          HY_Position_Record[11] = Zoom_Position & 0xff;                                                                      
                          EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),HY_Position_Record,Preset_Bytes_NUN);
                      }      
                }break;
                case Scan_Border_Right:
                {
                              u16 scan_id;
                      scan_id = (Down_Data & 0xff);
                      if (scan_id < 6)
                      {
                                      Sweep[scan_id].Zoom_Position_Right = Zoom_Position;
                          Sweep[scan_id].border_right = Pan_Real_Angle;
                          HY_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
                          HY_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
                          HY_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
                          HY_Position_Record[12] =  Zoom_Position >> 8;
                          HY_Position_Record[13] = Zoom_Position & 0xff;
                          EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),HY_Position_Record,Preset_Bytes_NUN);
                      }
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
                                  if(!(Down_Data & 0xff00))
                                  {
                                      Sweep[scan_id].speed = Speed_Level_Pan(0x0F00,0);
                                      HY_Position_Record[3] = Sweep[scan_id].speed;
              //                      EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),HY_Position_Record,Preset_Bytes_NUN);                  
                                  }
                                  else //进入则说明有速度设置，将data中的速度进行存储
                                  {
                                    Sweep[scan_id].speed = Speed_Level_Pan(Down_Data,0);
                          HY_Position_Record[3] = Sweep[scan_id].speed;
              //             EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),HY_Position_Record,Preset_Bytes_NUN);
                                  }
                                  //执行id
                                  if(EEPRom_ReadOneByte(EEPROM_Parameter,(scan_id + 230 - 1) * Preset_Bytes_NUN) == 0xaa)
                                  {
                          EEPRom_ReadDataLen(EEPROM_Parameter,(scan_id + 230),HY_Position_Extract,Preset_Bytes_NUN);
                          Sweep[scan_id].border_right = (double)(((HY_Position_Extract[7] | 0x0000) << 16) + ((HY_Position_Extract[8] | 0x0000) << 8) + HY_Position_Extract[9]) / 1000.0;
                          Sweep[scan_id].border_left = (double)(((HY_Position_Extract[4] | 0x0000) << 16) + ((HY_Position_Extract[5] | 0x0000) << 8) + HY_Position_Extract[6]) / 1000.0;
              //            Sweep[scan_id].speed = HY_Position_Extract[3];
                          Sweep[scan_id].time_left = HY_Position_Extract[1];
                          Sweep[scan_id].time_right = HY_Position_Extract[2];
                                      Sweep[scan_id].Zoom_Position_Left = (HY_Position_Extract[10]  << 8) + HY_Position_Extract[11];
                                      Sweep[scan_id].Zoom_Position_Right = (HY_Position_Extract[12] << 8) + HY_Position_Extract[13];
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
                          Scan_Num = 2;
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
                      HY_Position_Record[1] = Sweep[scan_id].time_left ;
                      HY_Position_Record[2] = Sweep[scan_id].time_right ;
                      HY_Position_Record[3] = Sweep[scan_id].speed;
                      HY_Position_Record[4] = (u32)(Sweep[scan_id].border_left * 1000) >> 16;
                      HY_Position_Record[5] = (u32)(Sweep[scan_id].border_left * 1000) >> 8;
                      HY_Position_Record[6] = (u32)(Sweep[scan_id].border_left * 1000);
                      HY_Position_Record[7] = (u32)(Sweep[scan_id].border_right * 1000) >> 16;
                      HY_Position_Record[8] = (u32)(Sweep[scan_id].border_right * 1000) >> 8;
                      HY_Position_Record[9] = (u32)(Sweep[scan_id].border_right * 1000);
                            HY_Position_Record[12] =  Zoom_Position >> 8;
                      HY_Position_Record[13] = Zoom_Position & 0xff;
                              HY_Position_Record[10] = Zoom_Position >> 8;
                      HY_Position_Record[11] = Zoom_Position & 0xff;  
                      EEPRom_WriteDataLen(EEPROM_Parameter,(scan_id + 230),HY_Position_Record,Preset_Bytes_NUN);
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
                          //可以开始学习了
                          StartML = 0;
                          Tracking[StartML].Start.Pan = Pan_Real_Angle;
                          Tracking[StartML].Start.Tilt = Tilt_Real_Angle;
                          Tracking[StartML].Stop.Pan = Pan_Real_Angle;
                          Tracking[StartML].Stop.Tilt = Tilt_Real_Angle;
                          Tracking[StartML].Zoom = Zoom_Position;
                          Tracking[StartML].Speed = 0x2020;
                          Tracking[StartML].Direction = 0x0000;           
                          Tracking[0].id = Down_Data & 0xff;//记录初始点数据
                          StartML ++;
                          Tracking[StartML].Start.Pan = Pan_Real_Angle;
                          Tracking[StartML].Start.Tilt = Tilt_Real_Angle;
                          Tracking[StartML].Stop.Pan = Pan_Real_Angle;
                          Tracking[StartML].Stop.Tilt = Tilt_Real_Angle;
                          Tracking[StartML].Zoom = Zoom_Position;
                          Tracking[StartML].Speed = 0x2020;
                          Tracking[StartML].Direction = 0x0000;           
                          StartML ++;
                      }break;
                      case Imitate_Stop:
                      {
                          Tracking[0].id = Down_Data & 0xff;
                          for(i = 0;i <= StartML;i ++)
                          {
                              HY_Position_Record[1] = StartML;
                              HY_Position_Record[2] = (u16)(Tracking[i].Start.Pan * 100) >> 8;
                              HY_Position_Record[3] = (u16)(Tracking[i].Start.Pan * 100) & 0xff;
                              HY_Position_Record[4] = (u16)(Tracking[i].Start.Tilt * 100) >> 8;
                              HY_Position_Record[5] = (u16)(Tracking[i].Start.Tilt * 100) & 0xff;
                              HY_Position_Record[6] = Tracking[i].Direction >> 8;
                              HY_Position_Record[7] = Tracking[i].Direction & 0xff;
                              HY_Position_Record[8] = Tracking[i].Speed >> 8;
                              HY_Position_Record[9] = Tracking[i].Speed & 0xff;
                              HY_Position_Record[10] = Tracking[i].Zoom >> 8;
                              HY_Position_Record[11] = Tracking[i].Zoom & 0xff;
                              HY_Position_Record[12] = (u16)(Tracking[i].Stop.Pan * 100) >>8;
                              HY_Position_Record[13] = (u16)(Tracking[i].Stop.Pan * 100) & 0xff;
                              HY_Position_Record[14] = (u16)(Tracking[i].Stop.Tilt * 100) >>8;
                              HY_Position_Record[15] = (u16)(Tracking[i].Stop.Tilt * 100) & 0xff;
                              EEPRom_WriteDataLen(EEPROM_Parameter,1850 + 25 * Tracking[0].id + i,HY_Position_Record,Preset_Bytes_NUN);
                          }
                          StartML = -1;            
                      }break;
                      //学习路径执行
                      case Imitate_start:
                      {
                          j = 0;
                          Tracking[0].id = Down_Data & 0xff;
                              EEPRom_ReadDataLen(EEPROM_Parameter,1850 + 25 * Tracking[0].id,HY_Position_Extract,Preset_Bytes_NUN);
                              j = HY_Position_Extract[1];
                              for (i = 0 ;i <= j ;i ++)
                              {
                                  EEPRom_ReadDataLen(EEPROM_Parameter,1850 + 25 * Tracking[0].id + i,HY_Position_Extract,Preset_Bytes_NUN);
                                  Tracking[i].Start.Pan = (double)(((HY_Position_Extract[2] << 8) + HY_Position_Extract[3]) / 100.0);
                                  Tracking[i].Start.Tilt = (double)(((HY_Position_Extract[4] << 8) + HY_Position_Extract[5]) / 100.0);
                                  Tracking[i].Direction = (HY_Position_Extract[6] << 8) + HY_Position_Extract[7];
                                  Tracking[i].Speed = (HY_Position_Extract[8] << 8) + HY_Position_Extract[9];
                                  Tracking[i].Zoom = (HY_Position_Extract[10] << 8) + HY_Position_Extract[11];
                                  Tracking[i].Stop.Pan = (double)(((HY_Position_Extract[12] << 8) + HY_Position_Extract[13]) / 100.0);
                                  Tracking[i].Stop.Tilt = (double)(((HY_Position_Extract[14] << 8) + HY_Position_Extract[15]) / 100.0); 
                              }
                              Special_Function = Tracking_ON;
                              Pan_Target_Angle = Tracking[0].Start.Pan;
                              Pan_Motion_Mode = Location_Motion;
                              Place_InOut_Pan = OutPlace;
                              Tilt_Target_Angle = Tracking[0].Start.Tilt;
                              Tilt_Motion_Mode = Location_Motion;
                              Place_InOut_Tilt = OutPlace;
                              Tracking[0].counter = j;
                              Tracking_Counter = 0;
                              Tracking_State = 0;
                              
                              send_data[0] = 0x81;
                              send_data[1] = 0x01;
                              send_data[2] = 0x04;
                              send_data[3] = 0x47;
                              send_data[4] = Tracking[0].Zoom >> 12;
                              send_data[5] = (Tracking[0].Zoom & 0x0f00) >> 8;
                              send_data[6] = (Tracking[0].Zoom & 0x00f0) >> 4;
                              send_data[7] = Tracking[0].Zoom & 0x000f;
                              send_data[8] = 0xFF; 
                              send_len = 9;
                              Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,send_data,send_len);            
                      }
                      //水果皮功能
                      case Skin_Direction_Set:
                      {
                          //获取id号
                          Fruit.Id = Down_Data & 0xFF;
                          //获取方向
                          Fruit.Direction = Down_Data >>8;
                          //存E方中
                          HY_Position_Record[1] = Fruit.Id;
                          HY_Position_Record[2] = Fruit.Direction;
                          if(EEPRom_ReadOneByte(EEPROM_Parameter,(Fruit.Id + Skin_Fruit -1  * Preset_Bytes_NUN) == 0xaa))
                    {
                         EEPRom_ReadDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),HY_Position_Extract,Preset_Bytes_NUN);  //先读出来上下限位再存进去 覆写防止丢失
                                      HY_Position_Record[3] = HY_Position_Extract[3];
                                      HY_Position_Record[4] = HY_Position_Extract[4];
                                      HY_Position_Record[5] = HY_Position_Extract[5];
                                      HY_Position_Record[6] = HY_Position_Extract[6];
                                      HY_Position_Record[7] = HY_Position_Extract[7];
                                      HY_Position_Record[8] = HY_Position_Extract[8];             
                          }
                          EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),HY_Position_Record,Preset_Bytes_NUN);
                      }break;
                      case Skin_UP_Set:
                      {
                          //获取id号
                          Fruit.Id = Down_Data & 0xFF;
                          //保存当前垂直坐标，存为上限位
                          Fruit.Up = Tilt_Real_Angle;
                          HY_Position_Record[3] = (u32)(Fruit.Up * 1000) >> 16;
                    HY_Position_Record[4] = (u32)(Fruit.Up * 1000) >> 8;
                    HY_Position_Record[5] = (u32)(Fruit.Up * 1000);
                          EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),HY_Position_Record,Preset_Bytes_NUN);
                      }break;
                      case Skin_Down_Set:
                      {
                          //获取id号
                          Fruit.Id = Down_Data & 0xFF;
                          Fruit.Down = Tilt_Real_Angle;
                          HY_Position_Record[6] = (u32)(Fruit.Down * 1000) >> 16;
                    HY_Position_Record[7] = (u32)(Fruit.Down * 1000) >> 8;
                    HY_Position_Record[8] = (u32)(Fruit.Down * 1000);
                          EEPRom_WriteDataLen(EEPROM_Parameter,(Fruit.Id + Skin_Fruit),HY_Position_Record,Preset_Bytes_NUN);
                      }

      }
}

#endif

