#include "TaskUSART1.h"
#include "TaskCAN.h"
#include "CRC16.h"
#include "stmflash.h"
#include "TaskUSART3.h"
#include "hardware.h"
#include "TaskEncoder.h"
float abc = 0;
//*********************************************
//串口1通讯任务处理
//*********************************************
//接收任务
void Task_USART1_RX(void *p)
{
    OS_ERR err;
//    uint32_t BAUD = 9600;
    p = p;
//    GET_ID_GPIO_INIT();
//    OSTimeDlyHMSM(0,0,0,50);
//    BAUD = Get_ID();
    USART1_INT_Init(USART1_BAND);
//    USART1_INT_Init(9600);
    OSSemCreate ((OS_SEM*   )&Sem_USART1_RX,
                 (CPU_CHAR* )"USART1_RX",
                 (OS_SEM_CTR)0,
                 (OS_ERR*   )&err);
    while(1)
    {
        OSSemPend(&Sem_USART1_RX,0,OS_OPT_PEND_BLOCKING,0,&err);
//        abc ++;
//        Pend_Information_Function(&USART1_RX_BOX,USART1_RX_Function);
        USART1_RX_Function();
//        OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
    }
}
//*********************************************
//接收命令解析
u16 USART1_byte_num = 0;
u8 USART1_byte_buf[20] = {0};
u8 Reply[6] = {0x02,0x01,0xb0,0x00,0x00,0x00};
void USART1_RX_Function(void)
{
    //Q_ERR err_q;
    u32 data[5] = {0};
    for(; USART1_dispose_data_pointer != USART1_data_pointer;)
    {
        USART1_dispose_data_pointer = USART1_dispose_data_pointer % DMA_USART1_RX_BUF_SIZE;
        if (USART1_byte_num == 0)
        {
            if(USART1_RX_Buf[USART1_dispose_data_pointer] == 0xff || USART1_RX_Buf[USART1_dispose_data_pointer] == 0x99)
            {
                USART1_byte_buf[USART1_byte_num ++] = USART1_RX_Buf[USART1_dispose_data_pointer];
            }
            else
            {
                USART1_byte_num = 0;
            }
        }
        else
        {
            USART1_byte_buf[USART1_byte_num ++] = USART1_RX_Buf[USART1_dispose_data_pointer];
					#ifdef SH
            if (USART1_byte_buf[1] == 0x01 || USART1_byte_buf[1] == 0x0b || USART1_byte_buf[1] == 0x0a)
					#else
						if (USART1_byte_buf[1] > 0x00 && USART1_byte_buf[1] < 0x0f)
					#endif
            {
                PT_END_ID = USART1_byte_buf[1];
                //USART1_byte_num ++;
                if(USART1_byte_num == 7)
                {
                    //USART1_byte_buf[USART1_byte_num] = USART1_RX_Buf[USART1_dispose_data_pointer ++];
										if((USART1_byte_buf[1] == 0x01 && USART1_byte_buf[2] == 0x00 && USART1_byte_buf[3] == 0xC1 && USART1_byte_buf[4] == 0x02))
                    {										
                        data[0] = upgrade + Local_CAN_StdId;
//                        STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
                        Delay_xMS(100);
                        STMFLASH_Write(Upgrade_CAN_ID,data,1);//????CANх??????
                        Delay_xMS(100);
                        iap_load_app(0x08000000);
                    }
									  if(USART1_byte_buf[1] == 0x0a && USART1_byte_buf[2] == 0xb1 && USART1_byte_buf[3] == 0x07)
                    {
                        data[0] = upgrade + Local_CAN_StdId;
                        //   APP_GOTO_IAP(Upgrade_CAN_ID,data,1);
                        //   data[1] = pan_track_offset;
                        //   data[2] = tilt_track_offset;
                        //   STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Current,Tilt_Encoder_Position_Current);
                        //   Encoder_Position_Storage();
//                        STMFLASH_Write_1(track_offect,Pan_Encoder_Position_Save,Tilt_Encoder_Position_Save,encoder_counter);
                        Delay_xMS(100);
                        STMFLASH_Write(Upgrade_CAN_ID,data,1);//????CANх??????
                        Delay_xMS(100);
                        iap_load_app(0x08000000);
                    }
                    else if(USART1_byte_buf[6] == ((USART1_byte_buf[1] + USART1_byte_buf[2] + USART1_byte_buf[3] + USART1_byte_buf[4] + USART1_byte_buf[5]) & 0xff))
                    {
                        if((USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x0051
                                || (USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x0053
                                || (USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x004b
                                || (USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x004d)
                        {
                            Angle_Accuracy_Flag = 0;
                            Angle_Decimal = 100.00;
                        }
                        Post_Information_Function(Sem_Analyze,&Analyze_BOX,(USART1_byte_buf[2] << 8) + USART1_byte_buf[3],(USART1_byte_buf[4] << 8) + USART1_byte_buf[5]);
                        USART1_First_Frame_Flag = 1;//????????????
                        USART1_byte_num = 0;
                    }
                }
                else if (USART1_byte_num == 8)
                {
                    USART1_byte_buf[USART1_byte_num] = USART1_RX_Buf[USART1_dispose_data_pointer];
                    if(USART1_byte_buf[7] == ((USART1_byte_buf[1] + USART1_byte_buf[2] + USART1_byte_buf[3] + USART1_byte_buf[4] + USART1_byte_buf[5]+ USART1_byte_buf[6] ) & 0xff))
                    {
                        if((USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x0051
                                || (USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x0053
                                || (USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x004b
                                || (USART1_byte_buf[2] << 8) + USART1_byte_buf[3] == 0x004d)
                        {
                            Angle_Accuracy_Flag = 1;
                            Angle_Decimal = 100.000;
                        }
                        Post_Information_Function(Sem_Analyze,&Analyze_BOX,(USART1_byte_buf[2] << 8) + USART1_byte_buf[3],(USART1_byte_buf[4] << 8) + (USART1_byte_buf[5]));
                        USART1_First_Frame_Flag = 1;//????????????
                    }
                    USART1_byte_num = 0;
                }
            }
            else
            {
                USART1_byte_num = 0;
                return;
            }
        }
        USART1_dispose_data_pointer ++;
        USART1_dispose_data_pointer = USART1_dispose_data_pointer % DMA_USART1_RX_BUF_SIZE;
    }
}
Alarm_Data Alarm[7] = {1,1,1,1};
void Alarm_Function_Analyze(void)
{
    //Q_ERR err_q;
    u16 Down_CMD = 0;
    u32 Down_Data = 0;
    int i;
    Down_CMD = Alarm_BOX.Memory_command[Alarm_BOX.extract_finger];
    Down_Data = Alarm_BOX.Memory_Data[Alarm_BOX.extract_finger];
    switch(Down_CMD)
    {
    case Alarm_Output:
        AlarmOut1 = Down_Data >> 8;
        AlarmOut2 = Down_Data & 0xff;
        break;
    case Alarm_Input_Setup:
        for (i = 0 ; i <= 8 ; i ++)
        {
            Alarm[i].enable = ((1 << i) & (Down_Data & 0xff));//报警功能是否开启 0开启 1关闭
            Alarm[i].enable >>= i;
            Alarm[i].type = ((1 << i) & (Down_Data >> 8));//报警输入类型设置。常开or常闭
            Alarm[i].type >>= i;
        }
        break;
    default:
        break;
    }
}
