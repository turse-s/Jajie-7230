#include "TaskLedState.h"
//#include "TaskUSART3.h"
//#include "TaskUSART1.h"
#include "stmflash.h"
#include "hardware.h"
#include "TaskUSART_TX.h"
#include "CRC16.h"
#include "TaskUSART1.h"
//static u8 Led3_State_Switch = 0;
//static u8 Led3_State_Count = 0;
//static u8 Led3_Breathe_PWM = 0;
u16 VISCA_FLAG = 0,key1 = 0;
extern u8 Pan_Angle_Data[7];
u16 id_data = 0;
void Task_LedState(void *p)
{
    OS_ERR err;
    int i;
    int send_countdown = 3;
//      int warning_on = OFF;
//    u32 upgrade_state = 0;
//    u32 data[1] = {0};
    u8 send_len = 0;
    u8 send_data[6] = {0};
    u8 USART_x = 0;
//    u16 crc_data = 0;
    p = p;
//    IWDG_Init(IWDG_Prescaler_256,1000);//8 * 1000ms = 8s
//    GPIO_PORT_INIT(LED1_port);
//    GPIO_PORT_INIT(LED2_port);
//    Led3_Breathe_Init(100-1,3600-1);//10ms周期
//    LED3 = OFF;
    LED2 = OFF;
//    Relay1 = ON;
//    GPIO_PORT_INIT_IN(Optocoupler_port);
//    GPIO_PORT_INIT(Relay_port);
//    Optocoupler_Sem = PBin(7);
//    upgrade_state = STMFLASH_ReadWord(0x08010000) & 0xffff0000;
//
//    if(upgrade_state == 0x00100000)
//    {
////            pan_track_offset = STMFLASH_ReadWord(0x08010004);
////            tilt_track_offset = STMFLASH_ReadWord(0x08010008);
//
//        send_data[0] = 0x02;
//        send_data[1] = 0x02;
//        send_data[2] = 0xb1;
//        send_data[3] = 0x00;
//        crc_data = CRC_16(send_data,4);
//        send_data[4] = crc_data & 0xff;
//        send_data[5] = (crc_data >> 8) & 0xff;
//        send_len = 6;
//        if(Return_Mode == Return_Query_485)
//        {
//            USART_x = 1;
//            Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);
//        }
//        else
//        {
//            USART_x = 2;
//            Post_Information_Function_TX(Sem_USART_TX,&USART2_TX_BOX,USART_x,send_data,send_len);
//        }
//        data[0] = 0xffff0000;
//        STMFLASH_Write(0x08010000,data,1);//记录当前CAN信道和升级标识
//    }
    while(1)
    {
        //*****************************
        LED2 = OFF;
//        LED3 = ~LED3;
        Alarm[0].status = PBin(7);
        for (i = 0; i < 8; i ++)
        {
            if(Alarm[i].enable == ON)//8路报警使能
            {
                if(Alarm[i].status != Alarm[i].type)//8路报警当前状态和默认状态相比
                {
                    if(send_countdown != 0)//报警默认发三次
                    {
                        USART_x = 1;
                        send_len = 6;
                        send_data[0] = 0xFA;
                        send_data[1] = 0x01;
                        send_data[2] = 0x07;
                        send_data[3] = (1 << i);
                        send_data[4] = 0x00;
                        send_data[5] = Check_Sum(send_data,0,4);
                        Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);//发送报警讯息
                        delay_ms(100);
                        Alarm[i].alert = ON;
                        send_countdown --;
                    }
                }
                else
                {
                    if (Alarm[i].alert == ON)
                    {
                        USART_x = 1;
                        send_len = 6;
                        send_data[0] = 0xFA;
                        send_data[1] = 0x01;
                        send_data[2] = 0x07;
                        send_data[3] = 0x00;
                        send_data[4] = 0x00;
                        send_data[5] = Check_Sum(send_data,0,4);
                        Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);//发送撤防讯息
                        delay_ms(100);
                        Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);//发送撤防讯息
                        Alarm[i].alert = OFF;
                        send_countdown = 3;//报警发送重新充能
                    }
                }
            }
        }
        if (AlarmOut1 == ON)//1路报警输出
        {
            PBout(6) = OFF;
        }
        else
        {
            PBout(6) = ON;
        }
        if (AlarmOut2 == ON)//2路报警输出
        {
        }
        if(USART3_Protocol_Type != Protocol_VISCA)
        {
            if(USART1_First_Frame_Flag && !VISCA_CAM_Zoom_Flag)
            {
                VISCA_CAM_Zoom_Flag = 1;
//                USART3_ENABLE();
                send_len = 5;
                memcpy(send_data,CAM_ZoomPosInq,send_len);
                USART_x = 3;
                Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,USART_x,send_data,send_len);
            }
            OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,&err);
        }
        else
        {
            OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
        }
    }
}
void Led3_Breathe(void)
{
//  int i = 0;
//    OS_ERR err;
//    if(Led3_State_Switch)
//    {
//        Led3_State_Count --;
//        if(Led3_State_Count == 0)
//        {
//            Led3_State_Switch = 0;
//        }
//    }
//    else
//    {
//        Led3_State_Count ++;
//        if(Led3_State_Count == 120)
//        {
//            Led3_State_Switch = 1;
//        }
//    }
//    Led3_Breathe_PWM = Led3_State_Count;
//    if(Led3_State_Count >= 100)
//    {
//        Led3_Breathe_PWM = 100;
//    }
//    TIM_SetCompare4(TIM1,Led3_Breathe_PWM);
//    OSTimeDlyHMSM(0,0,0,16,OS_OPT_TIME_HMSM_STRICT,&err);
}
