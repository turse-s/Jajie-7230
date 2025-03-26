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
#include "RockerControl.h"
#include "SpecialFunctions.h"
u16 Tim_Com = 0;
u16 Tim_Com2 = 0;
u16 Tim_ML = 0;
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
				SpecialFunctions();
        OSTimeDlyHMSM(0,0,0,Task_Timer_Cycle_Time,OS_OPT_TIME_HMSM_STRICT,&err);
    }
}
