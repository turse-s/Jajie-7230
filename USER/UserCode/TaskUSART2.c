#include "TaskUSART2.h"
#include "TaskCAN.h"
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
#include "RockerControl.h"
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
				USART2_RX_Function();
#ifdef ZS
        if(U2_switch)
        {
            USART2_RX_Function();
        }
#else
//				Pend_Information_Function(&USART2_RX_BOX,USART2_RX_Function);
#endif
    }
}
u16 USART2_byte_num = 0;
u8 USART2_byte_buf[300];
//void USART2_RX_Function(void)
//{
//    //数据保存
//   for(;USART2_data_pointer != USART2_dispose_data_pointer;)
//    {
//        USART2_dispose_data_pointer = USART2_dispose_data_pointer % DMA_USART2_RX_BUF_SIZE;
//        if(USART2_byte_num == 0)
//        {
//            if(USART2_RX_Buf[USART2_dispose_data_pointer] == 0xff)
//            {
//                USART2_byte_buf[USART2_byte_num] = USART2_RX_Buf[USART2_dispose_data_pointer];
//                USART2_byte_num ++;
//            }
//        }
//        else
//        {
//            USART2_byte_buf[USART2_byte_num] = USART2_RX_Buf[USART2_dispose_data_pointer];
//            USART2_byte_num ++;
//            if(USART2_byte_num > 6)
//            {
//                if(USART2_byte_buf[6] == ((USART2_byte_buf[1] + USART2_byte_buf[2] + USART2_byte_buf[3] + USART2_byte_buf[4] + USART2_byte_buf[5]) & 0xff))
//                {
//                    USART2_RX_Analyze();
//                }
//                USART2_byte_num = 0;
//            }
//        }
//        USART2_dispose_data_pointer ++;
//        USART2_dispose_data_pointer = USART2_dispose_data_pointer % DMA_USART2_RX_BUF_SIZE;
//    }
//}

void USART2_RX_Function(void)
{
		u32 data[5] = {0};
    //数据保存
    for(; USART2_data_pointer != USART2_dispose_data_pointer;)
    {
        USART2_dispose_data_pointer = USART2_dispose_data_pointer % DMA_USART2_RX_BUF_SIZE;
        if(USART2_byte_num == 0)
        {
            if(USART2_RX_Buf[USART2_dispose_data_pointer] == 0xff || USART2_RX_Buf[USART2_dispose_data_pointer] == 0x99 )
            {
                USART2_byte_buf[USART2_byte_num] = USART2_RX_Buf[USART2_dispose_data_pointer];
                USART2_byte_num ++;
            }
        }
        else
        {
            USART2_byte_buf[USART2_byte_num] = USART2_RX_Buf[USART2_dispose_data_pointer];
            USART2_byte_num ++;
					#ifdef SH
            if (USART2_byte_buf[1] == 0x01 || USART2_byte_buf[1] == 0x0b)
					#else
						if (USART2_byte_buf[1] > 0x00 && USART2_byte_buf[1] <0x0f)
					#endif
            {
                if(USART2_byte_num > 6)
                {
                    if(USART2_byte_buf[6] == ((USART2_byte_buf[1] + USART2_byte_buf[2] + USART2_byte_buf[3] + USART2_byte_buf[4] + USART2_byte_buf[5]) & 0xff))
                    {
#ifdef ZS03
											if ( 	(USART2_byte_buf[2] << 8) + USART2_byte_buf[3] == ZOOM_Tele || 
														(USART2_byte_buf[2] << 8) + USART2_byte_buf[3] == ZOOM_Wide || 
														(USART2_byte_buf[2] << 8) + USART2_byte_buf[3] == FOCUS_Far || 
														(USART2_byte_buf[2] << 8) + USART2_byte_buf[3] == FOCUS_Near ||
														(USART2_byte_buf[2] << 8) + USART2_byte_buf[3] == Query_ZOOM)
											{
														if (USART2_byte_buf[1] != 0x01)
														{
																USART2_byte_num = 0;
																return;
														}
											}
#endif									                                      
											//填充结构体
											Analyze_BOX2.Memory_command[Analyze_BOX2.memory_finger]=(USART2_byte_buf[2] << 8) + USART2_byte_buf[3];
                      Analyze_BOX2.Memory_Data[Analyze_BOX2.memory_finger]=(USART2_byte_buf[4] << 8) + USART2_byte_buf[5];
                      Analyze_BOX2.Statistic_number ++;
                      Analyze_BOX2. memory_finger ++;
                      if(Analyze_BOX2.memory_finger >= Memory_Finger_NUM)
                      {
                          Analyze_BOX2.memory_finger = 0;
                      }		
#ifdef INDU2
                            Pend_Information_Function(&Analyze_BOX2,USART2_RX_Analyze);
#else
														Post_Information_Function(Sem_Analyze,&Analyze_BOX,(USART2_byte_buf[2] << 8) + USART2_byte_buf[3],(USART2_byte_buf[4] << 8) + (USART2_byte_buf[5]));
#endif														

                    }
										if((USART2_byte_buf[1] == 0x01 && USART2_byte_buf[2] == 0x00 && USART2_byte_buf[3] == 0xC1 && USART2_byte_buf[4] == 0x02))
                    {										
                        data[0] = upgrade + Local_CAN_StdId;
//                        STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
                        Delay_xMS(100);
                        STMFLASH_Write(Upgrade_CAN_ID,data,1);//????CANх??????
                        Delay_xMS(100);
                        iap_load_app(0x08000000);
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
        //Q_ERR err_q;
        u16 Down_CMD = 0;
        u32 Down_Data = 0;
			  Down_CMD = Analyze_BOX2.Memory_command[Analyze_BOX2.extract_finger];
        Down_Data = Analyze_BOX2.Memory_Data[Analyze_BOX2.extract_finger];
        CMD_Analyze(Down_CMD,Down_Data,2);
}
//*************************************************************************
u8 Origin_Machine_Data_U2[8];
