#include "TaskEncoder.h"
#include "AS5600.h"
#include "MT6825.h"
#include "24LCxx.h"
#include "TaskUSART1.h"
//#include "stmflash.h"
#include "TaskAnalyze.h"
#include "TaskMotor.h"
#include "hardware.h"
#include "stmflash.h"
#include "CRC16.h"
#include "Filter.h"
u8 top = 0;
u8 middle = 0;
u8 bottom = 0;
u8 check_flag = 0;
u32 count = 0;
u8 count_over = 0;
u8 Pan_Section_CAP_Counter = 0;
u8 Tilt_Section_CAP_Counter = 0;
u8 Special_Function_Pan = 0;
u8 Special_Function_Tilt = 0;
u8 Special_Function_Pan_Over = 0;
u8 Special_Function_Tilt_Over = 0;
u8 tt = 0;
u16 Pan_M = 0;
u8 init_p_ready = 1;
u8 init_t_ready = 1;
u8 Encoder_Check_Pan(void)
{
    u8 ep = 0;
    Pan_IIC_Init();
    SPI_PORT_INIT(Pan_SPI_port);
    if((Encoder_SPI_Pan(IC_MU_CMD_SDAD_Status) & 0x000f0000) == 0x00020000)//IC_MU
    {
        ep = 3;
        Pan_Encoder_Range = 262144;
        Pan_Angle_Accuracy = Circle_Value / Pan_Encoder_Range;
        Pan_SetAngle_RetardSection = ICMU_RS_P;
        ratio_add = 2;
        ratio_subtract_pan = 2;
        Pan_Encoder_Data_Last = Encoder_SPI_Pan(IC_MU_CMD_SDAD_Transmission) & 0x00ffffff >> 6;
    }
    else if(MT6825_Pan() != 0xee000000 && MT6825_Pan() != 0xffffffff)//MT6825
    {
        ep = 4;
        Pan_Encoder_Range = 262144;
        Pan_Angle_Accuracy = 0.01;
        Pan_SetAngle_RetardSection = MT6825_RS_P;
        ratio_add = 2;
        ratio_subtract_pan = 0.5;
        Pan_Encoder_Data_Last = MT6825_Pan();
    }
#ifndef FOC
    else if(Pan_get_Angle() != 0xeeee)//AS5600
    {
        ep = 2;
        Pan_Encoder_Range = 4096;   //姝や负缂栫爜鍣ㄧ殑鏈�澶ф暟鍊?
        Pan_Angle_Accuracy = Circle_Value / Pan_Encoder_Range;  //姝ゅ寰楀埌缂栫爜鍣ㄥ搴旂殑绮惧害锛堝垎杈ㄧ巼锛?
        Pan_SetAngle_RetardSection = AS5600_RS_P;   //姝ゅ璁剧疆鍑忛�熻搴︼紝浣庝簬杩欎竴瑙掑害浜戝彴寮�濮嬭繘琛屾瘮渚嬪噺閫?
        ratio_add = 2;
        ratio_subtract_pan = 4;
        Pan_Encoder_Data_Last = Pan_get_Angle();
    }
#endif		
    else
    {
        ep = 0;
    }
    return ep;
}
u8 Encoder_Check_Tilt(void)
{
    u8 et = 0;
		Tilt_IIC_Init();	
#ifdef FOC
				SPI_PORT_INIT_Tilt(Tilt_SPI_port);
//			SPIx_Init(SPI3,GPIOC,GPIO_Pin_10,GPIO_Pin_11,GPIO_Pin_12,GPIOA,GPIO_Pin_15,GPIO_PinSource10,GPIO_PinSource11,GPIO_PinSource12);//Tilt SPI int
#else
        SPI_PORT_INIT(Tilt_SPI_port);
#endif
    if((Encoder_SPI_Tilt(IC_MU_CMD_SDAD_Status) & 0x000f0000) == 0x00020000)//IC_MU
    {
        et = 3;
        Tilt_Encoder_Range = 262144;
        Tilt_Angle_Accuracy = Circle_Value / Tilt_Encoder_Range;
        Tilt_SetAngle_RetardSection = ICMU_RS_T;
        ratio_add = 2;
        ratio_subtract_tilt = 2;
        Tilt_Encoder_Data_Last = Encoder_SPI_Tilt(IC_MU_CMD_SDAD_Transmission) & 0x00ffffff >> 6;
    }
    else if(MT6825_Tilt() != 0xee000000 && MT6825_Tilt() != 0xffffffff)//MT6825
    {
        Tilt_Encoder_Data_Last = MT6825_Tilt();
        et = 4;
        Tilt_Encoder_Range = 262144;
        Tilt_Angle_Accuracy = 0.01;
        Tilt_SetAngle_RetardSection = MT6825_RS_T;
        ratio_add = 2;
        ratio_subtract_tilt = 1.2;
        Tilt_Encoder_Data_Last = MT6825_Tilt();
    }
#ifndef FOC
    else if(Tilt_get_Angle() != 0xeeee)//AS5600
    {
        et = 2;
        Tilt_Encoder_Range = 4096;
        Tilt_Angle_Accuracy = Circle_Value / Tilt_Encoder_Range;
        Tilt_SetAngle_RetardSection = AS5600_RS_T;
        ratio_add = 2;
        ratio_subtract_tilt = 4;
        Tilt_Encoder_Data_Last = Tilt_get_Angle();
    }
#endif
    else
    {
        et = 0;
    }
    return et;
}
u8 Encoder_Check_Pan_Flag = 0,Encoder_Check_Tilt_Flag = 0;
u8 run_flag = 0;
u32 data_all = 0;
static double x_last;
//JQueue_t Encoder_Position_Pan_A;
//JQueue_t Encoder_Position_Tilt_A;
//JQueue_t Encoder_Position_Pan_B;
//JQueue_t Encoder_Position_Tilt_B;
//u32 Encoder_Position_Pan_BUF_A[Encoder_Position_BUF_A_LEN];
//u32 Encoder_Position_Pan_BUF_B[Encoder_Position_BUF_B_LEN];
//u32 Encoder_Position_Tilt_BUF_A[Encoder_Position_BUF_A_LEN];
//u32 Encoder_Position_Tilt_BUF_B[Encoder_Position_BUF_B_LEN];
//u32 Encoder_Position_Pan_BUF_A_t[Encoder_Position_BUF_A_LEN];
//u32 Encoder_Position_Tilt_BUF_A_t[Encoder_Position_BUF_A_LEN];
Encoder_Position_Store_t Encoder_Position_Store;
int Encoder_Position_Pan_err = 0;
int Encoder_Position_Tilt_err = 0;
u8 pan_err_time = 0;//褰撳墠鍊煎嚭鐜拌烦鍙樼殑娆℃暟锛屽嵆缂撳瓨B璁℃弧鍛ㄦ湡鍐呯殑璺冲彉娆℃暟
u8 tilt_err_time = 0;
u32 P_D = 0;
u32 T_D = 0;
Filter_Data_t pan_data;
Filter_Data_t tilt_data;
void Task_Encoder(void *p)
{
    OS_ERR err;
//      Q_ERR err_q;
//    Pan_Calibration_Init();
//    Tilt_Calibration_Init();
//    u32 data[] = {0};
    double TCAU = 0;
    u32 upgrade_state = 0;
    u32 data[1] = {0};
    u8 send_len = 0;
    u8 send_data[6] = {0};
    u8 USART_x = 0;
    u16 crc_data = 0;
    p = p;
    while(Encoder_Check_Pan_Flag < 3)       //妫?娴?3娆＄紪鐮佸櫒绫诲瀷鎻愰珮鍙潬鎬?
    {
        Encoder_Check_Pan_Flag ++;
        pan_encoder_type = Encoder_Check_Pan();
        if(pan_encoder_type != 0)
        {
            Encoder_Check_Pan_Flag = 3;
        }
        delay_ms(100);
    }
    while(Encoder_Check_Tilt_Flag < 3)
    {
        Encoder_Check_Tilt_Flag ++;
        tilt_encoder_type = Encoder_Check_Tilt();
        if(tilt_encoder_type != 0)
        {
            Encoder_Check_Tilt_Flag = 3;
        }
        delay_ms(100);
    }
    delay_ms(100);
    Pan_Encoder_Data_Last = Pan_Encoder_Position(pan_encoder_type);
    Tilt_Encoder_Data_Last = Tilt_Encoder_Position(tilt_encoder_type);
    delay_ms(100);
    Pan_Encoder_Data_Last = Pan_Encoder_Position(pan_encoder_type);     //上电时Pan_Encoder_Data_Last为0，读两次数据更新Pan_Encoder_Data_Last为当前值并提高可靠性
    Tilt_Encoder_Data_Last = Tilt_Encoder_Position(tilt_encoder_type);  //???_Encoder_Data_Last为编码器原始值
    Pan_Encoder_Position_Current = Pan_Encoder_Data_Last;           //Pan_Encoder_Position_Current为编码器当前值，其是有last的值赋值过来的，由于读编码器是有时间间隔的所以这两个值有可能不同，此处是将其初始化所以两个值是相通的
    Pan_Encoder_Position_Save = Pan_Encoder_Data_Last;              //用于掉电存储的编码器值
    Tilt_Encoder_Position_Current = Tilt_Encoder_Data_Last;
    Tilt_Encoder_Position_Save = Tilt_Encoder_Data_Last;
    PWR_PVD_Init();
//    TIM6_Init(Motor_arr,Motor_psc);//50ms
//    TIM7_Init(Motor_arr,Motor_psc);//50ms
#ifndef FOC
		TIM4_Init(28080,0);
		TIM5_Init(28080,0);
		Motor_Tilt_Init();
    Motor_Tilt_PWM_Init(Motor_PWM_arr,Motor_PWM_psc);
		Motor_Pan_Init();
    Motor_Pan_PWM_Init(Motor_PWM_arr,Motor_PWM_psc);
#endif
    TIM10_Init(TIM10_ARR,TIM10_PSC);//1ms
    CAN1_Mode_Init(0,0,0x0b,3,0x00);

//    STMFLASH_Read(0x08010004,data,1);
//    Drive_Switch_ON;
//    UP_Switch_ON;
    Pan_Kp = P_Kp;
    Pan_Ki = P_Ki;
    Pan_Kd = P_Kd;
    Tilt_Kp = T_Kp;
    Tilt_Ki = T_Ki;
    Tilt_Kd = T_Kd;
    //********************************************************************
    //必须在编码器初始化完成之后！
    Parameters_Init();
		//Usart1_Send_Boot();
    upgrade_state = STMFLASH_ReadWord(0x08010000) & 0xffff0000;
    if(upgrade_state == 0x00100000)     //如果是从IAP升级过来的，则升级完毕进入APP后恢复上位机一个进入APP的信息
    {
//            pan_track_offset = STMFLASH_ReadWord(0x08010004);
//            tilt_track_offset = STMFLASH_ReadWord(0x08010008);
        send_data[0] = 0x02;
        send_data[1] = 0x02;
        send_data[2] = 0xb1;
        send_data[3] = 0x00;
        crc_data = CRC_16(send_data,4);
        send_data[4] = crc_data & 0xff;
        send_data[5] = (crc_data >> 8) & 0xff;
        send_len = 6;
        if(Return_Mode == Return_Query_485)
        {
            USART_x = 1;
            Post_Information_Function_TX(Sem_USART_TX,&USART1_TX_BOX,USART_x,send_data,send_len);
        }
        else
        {
            USART_x = 2;
            Post_Information_Function_TX(Sem_USART_TX,&USART2_TX_BOX,USART_x,send_data,send_len);
        }
        data[0] = 0xffff0000;       //清楚标志位保证只有iap升级完毕后进行一次上位机回复，之后就不会在回复了
        STMFLASH_Write(0x08010000,data,1);//记录当前CAN信道和升级标识
    }
//    FilterCreate(&Pan_Angle_Filter,10,Pan_Current_Angle_User);
//    FilterCreate(&Tilt_Angle_Filter,10,Tilt_Current_Angle_User);
    //********************************************************************
//    STM32Sector_Erase(0x08020000);
//    if(pan_encoder_type == 3 && tilt_encoder_type == 3)
//  Encoder_Init();
//  //*********************************************************
//    if(!run_flag)
//    {
//        run_flag = 1;
//        STM32Sector_Erase(track_offect);
//    }
//      pan_data.error = 370;
//          tilt_data.error = 370;//极限滤波上限
    pan_data.Main_Switch = 0;
    tilt_data.Main_Switch = 0;
    pan_data.length = 10;
    tilt_data.length = 10;//婊ゆ尝鏁扮粍闀垮害
    pan_data.resolution = (Circle_Value + 3) / Pan_Encoder_Range;
    tilt_data.resolution = (Circle_Value + 3) / Tilt_Encoder_Range;//鏋侀檺婊ゆ尝涓嬮檺
    //pan_data.counter = 10.0;
    //tilt_data.counter = 10.0;//鈥滄楠椻�濇鏁板�嶇巼
    pan_data.Disposable_Switch = 1;
    tilt_data.Disposable_Switch = 1;
    pan_data.Init_Ready = pan_data.length;
    tilt_data.Init_Ready = tilt_data.length;
    while(1)
    {
//        //*********************************************************************************************************
//        //编码器下线检测
//        if(Pan_Encoder_State == Encoder_Drop)
//        {
//            pan_encoder_type = Encoder_Check_Pan();
//            Pan_Motion_Mode = Rocker_Motion;
//            Pan_Target_Speed = 0;
//            Pan_Encoder_Data_Last = Pan_Encoder_Position(pan_encoder_type);
//            delay_ms(100);
//            Pan_Encoder_Data_Last = Pan_Encoder_Position(pan_encoder_type);
//            Pan_Encoder_Data_Poweroff = Pan_Encoder_Data_Last;
//            Parameters_Init();
//        }
//        if(Tilt_Encoder_State == Encoder_Drop)
//        {
//            tilt_encoder_type = Encoder_Check_Tilt();
//            Tilt_Motion_Mode = Rocker_Motion;
//            Tilt_Target_Speed = 0;
//            Tilt_Encoder_Data_Last = Tilt_Encoder_Position(tilt_encoder_type);
//            delay_ms(100);
//            Tilt_Encoder_Data_Last = Tilt_Encoder_Position(tilt_encoder_type);
//            Tilt_Encoder_Data_Poweroff = Tilt_Encoder_Data_Last;
//            Parameters_Init();
//        }
        //*******************************杩欓儴鍒嗕负姘村钩缂栫爜鍣ㄥ師濮嬫暟鎹幏鍙?********************************************************
        if(pan_encoder_type == 3)//IC-MU
        {
            P_D = Pan_Encoder_Position(pan_encoder_type);           //鑾峰彇缂栫爜鍣ㄥ師濮嬫暟鎹?
            if(P_D != 0xeeeeeeee)                                   //鏁版嵁鍚堢悊鎬у垽鏂?
            {
                Pan_Encoder_Position_Save = Pan_Encoder_Position_Current;       //鐢ㄥ綋鍓嶇紪鐮佸櫒鏁版嵁鏇存柊鎺夌數淇濆瓨缂栫爜鍣?
                Pan_Encoder_Position_Real = P_D;                                //Pan_Encoder_Position_Real涓哄綋鍓嶇紪鐮佸櫒鍘熷鏁版嵁
                Pan_Encoder_Position_Current = (Pan_Encoder_Position_Real + pan_track_offset * 8192) & 0x3ffff;     //鍘熷鏁版嵁+鍋忕Щ琛ュ伩鍊?*8192锛屽亸绉昏ˉ鍋垮?兼槸涓轰簡瑙ｅ喅ICMU璺抽亾闂鑰岃璁＄殑琛ュ伩鍊?
            }
            else
            {
            }
//
//          //鍏堝垽鏂紪鐮佸櫒鐘舵??
//          if((Encoder_SPI_Pan(IC_MU_CMD_SDAD_Status) & 0x000f0000) == 0x00020000)
//          {
//              P_D = Encoder_SPI_Pan(IC_MU_CMD_SDAD_Transmission);
//
//              Pan_Encoder_State = Encoder_Normal;
//              //缂栫爜鍣ㄧ姸鎬佹湁鏁?
//              Pan_Encoder_Position_Real = P_D;
//              //*********************************************************
//              //缂撳瓨A
//              J_Enqueue(&Encoder_Position_Pan_A,&Pan_Encoder_Position_Real,&err_q);//瀛樺偍姣忎竴涓紪鐮佸櫒鍊?
//
//              if(!J_GetQueueDataLen(&Encoder_Position_Pan_B))
//              {
//                  J_Enqueue(&Encoder_Position_Pan_B,&Pan_Encoder_Position_Real,&err_q);//瀛樺偍B
//              }
//              else
//              {
//                  //缂撳瓨B
//                  //鍒ゆ柇褰撳墠鍊兼槸鍚﹀皬浜嶣鐨勬渶鍚庝竴涓??
//                  if(abs(((int)((Pan_Encoder_Position_Real - Encoder_Position_Pan_BUF_B[J_GetQueueDataLen(&Encoder_Position_Pan_B) - 1]) << 14)) /16384) <= (int)Encoder_Sector_Position)
//                  {
//                      J_Enqueue(&Encoder_Position_Pan_B,&Pan_Encoder_Position_Real,&err_q);//瀛樺偍B
//                      if(J_GetQueueDataLen(&Encoder_Position_Pan_B) == Encoder_Position_BUF_B_LEN)//褰撳墠B缂撳瓨宸茬粡瀛樻弧锛屽垽鏂瑼锛圓鐨勫悗80锛夈?丅鐨勫亸宸槸鍚﹀ぇ浜庤烦鍖虹殑鏁村?嶆暟
//                      {
//                          J_GetAllData(&Encoder_Position_Pan_A,Encoder_Position_Pan_BUF_A_t,&err_q);
//                          //璁＄畻鍋忓樊绯绘暟
//                          pan_track_err = (int)((Encoder_Position_Pan_BUF_A_t[J_GetQueueDataLen(&Encoder_Position_Pan_A) - Encoder_Position_BUF_B_LEN - pan_err_time] - Encoder_Position_Pan_BUF_B[0]) << 14);
//                          pan_track_err /= 16384;
//                          pan_track_offset += pan_track_err / 8192 + (pan_track_err % 8192) / 4096;
//                          J_EmptyData(&Encoder_Position_Pan_B);//娓呯┖B
//                          pan_err_time = 0;
//                          J_Enqueue(&Encoder_Position_Pan_B,&Pan_Encoder_Position_Real,&err_q);//瀛樺偍B
//                      }
//                  }
//                  else//Bn > Bn-1
//                  {
//                      J_EmptyData(&Encoder_Position_Pan_B);//娓呯┖B
//                      pan_err_time ++;
//                      J_Enqueue(&Encoder_Position_Pan_B,&Pan_Encoder_Position_Real,&err_q);//瀛樺偍B
//                  }
//              }
//              //*********************************************************
//                Pan_Encoder_Position_Current = (Pan_Encoder_Position_Real + pan_track_offset * 8192) & 0x3ffff;
//          }
//          else
//          {
//              Pan_Encoder_State = Encoder_Drop;
//          }
        }
#ifndef FOC
        if(pan_encoder_type == 2)
        {
            P_D = Pan_get_Angle();      //P_D涓虹紪鐮佸櫒鍘熷鏁版嵁
            if(P_D != 0xeeee)           //鍘熷鏁版嵁鍚堢悊鎬ф鏌?
            {
                Pan_Encoder_Data = P_D; //
            }
        }
#endif
        if(pan_encoder_type == 4)
        {
            P_D = MT6825_Pan() & 0x3fff0;
#ifdef PT_HIAC_7280
            P_D = 0x3ffff - P_D;
#endif
            if((P_D & 0xee000000) != 0xee000000)
            {
                Pan_Encoder_Position_Real = P_D & 0xffffff;
                Pan_Encoder_Position_Current = Pan_Encoder_Position_Real;
            }
        }
        //***********************************杩欓儴鍒嗕负淇话缂栫爜鍣ㄥ師濮嬫暟鎹幏鍙?******************************************************
        if(tilt_encoder_type == 3)
        {
            T_D = Tilt_Encoder_Position(tilt_encoder_type);
            if(T_D != 0xeeeeeeee)
            {
                Tilt_Encoder_Position_Save = Tilt_Encoder_Position_Current;
                Tilt_Encoder_Position_Real = T_D;
                Tilt_Encoder_Position_Current = (Tilt_Encoder_Position_Real + tilt_track_offset * 8192) & 0x3ffff;
            }
//          //鍏堝垽鏂紪鐮佸櫒鐘舵??
//          if((Encoder_SPI_Tilt(IC_MU_CMD_SDAD_Status) & 0x000f0000) == 0x00020000)
//          {
//              T_D = Encoder_SPI_Tilt(IC_MU_CMD_SDAD_Transmission);
//
//              Tilt_Encoder_State = Encoder_Normal;
//              //缂栫爜鍣ㄧ姸鎬佹湁鏁?
//              Tilt_Encoder_Position_Real = T_D;
//              //*********************************************************
//              //缂撳瓨A
//              J_Enqueue(&Encoder_Position_Tilt_A,&Tilt_Encoder_Position_Real,&err_q);//瀛樺偍姣忎竴涓紪鐮佸櫒鍊?
//              if(!J_GetQueueDataLen(&Encoder_Position_Tilt_B))
//              {
//                  J_Enqueue(&Encoder_Position_Tilt_B,&Tilt_Encoder_Position_Real,&err_q);//瀛樺偍B
//              }
//              else
//              {
//                  //缂撳瓨B
//                  //鍒ゆ柇褰撳墠鍊兼槸鍚﹀皬浜嶣鐨勬渶鍚庝竴涓??
//                  if(abs((int)((Tilt_Encoder_Position_Real - Encoder_Position_Tilt_BUF_B[J_GetQueueDataLen(&Encoder_Position_Tilt_B) - 1]) << 14) /16384) <= Encoder_Sector_Position)
//                  {
//                      J_Enqueue(&Encoder_Position_Tilt_B,&Tilt_Encoder_Position_Real,&err_q);//瀛樺偍B
//                      if(J_GetQueueDataLen(&Encoder_Position_Tilt_B) == Encoder_Position_BUF_B_LEN)//褰撳墠B缂撳瓨宸茬粡瀛樻弧锛屽垽鏂瑼锛圓鐨勫悗80锛夈?丅鐨勫亸宸槸鍚﹀ぇ浜庤烦鍖虹殑鏁村?嶆暟
//                      {
//                          J_GetAllData(&Encoder_Position_Tilt_A,Encoder_Position_Tilt_BUF_A_t,&err_q);
//                          //璁＄畻鍋忓樊绯绘暟
//                          tilt_track_err = (int)((Encoder_Position_Tilt_BUF_A_t[J_GetQueueDataLen(&Encoder_Position_Tilt_A) - Encoder_Position_BUF_B_LEN - tilt_err_time] - Encoder_Position_Tilt_BUF_B[0]) << 14);
//                          tilt_track_err /= 16384;
//                          tilt_track_offset += tilt_track_err / 8192 + (tilt_track_err % 8192) / 4096;
//
//                          J_EmptyData(&Encoder_Position_Tilt_B);//娓呯┖B
//                          tilt_err_time = 0;
//                          J_Enqueue(&Encoder_Position_Tilt_B,&Tilt_Encoder_Position_Real,&err_q);//瀛樺偍B
//                      }
//                  }
//                  else//Bn > Bn-1
//                  {
//                      J_EmptyData(&Encoder_Position_Tilt_B);//娓呯┖B
//                      tilt_err_time ++;
//                      J_Enqueue(&Encoder_Position_Tilt_B,&Tilt_Encoder_Position_Real,&err_q);//瀛樺偍B
//                  }
//              }
//              //*********************************************************
//                Tilt_Encoder_Position_Current = (Tilt_Encoder_Position_Real + tilt_track_offset * 8192) & 0x3ffff;
//          }
//          else
//          {
//              Tilt_Encoder_State = Encoder_Drop;
//          }
        }
#ifndef FOC
        if(tilt_encoder_type == 2)
        {
            T_D = Tilt_get_Angle();
            if(T_D != 0xeeee)
            {
                Tilt_Encoder_Data = T_D;
            }
        }
#endif
        if(tilt_encoder_type == 4)
        {
            T_D = MT6825_Tilt() & 0x3fff0;
#ifdef PT_HIAC_7280
            T_D = 0x3ffff - T_D;
#endif
            if((T_D & 0xee000000) != 0xee000000)
            {
                Tilt_Encoder_Position_Real = T_D & 0xffffff;
                Tilt_Encoder_Position_Current = Tilt_Encoder_Position_Real;
            }
        }
//***********************************杩欓儴鍒嗕负姘村钩缂栫爜鍣ㄦ暟鎹鐞嗭紝瑙掑害杞崲绛?**********************************************************************
        switch(pan_encoder_type)
        {
//*******************************************************************************************
        case 1://AS5311
        {
        } break;
//*******************************************************************************************
#ifndef FOC				
        case 2://AS5600
        {
            //*******************************************************************
            //编码器出厂数据
            Pan_Current_Data = (Pan_Encoder_Data - Pan_Machine_Origin & 0xffff) & 0xfff;                //缂栫爜鍣ㄥ師濮嬫暟鎹噺鍘绘満姊板師鐐规暟鎹紝杩欓噷寰楀埌鐨勬槸鐩稿浜庢満姊板師鐐圭殑鏁版嵁锛屽鏈烘鍘熺偣涓?100锛屽綋鍓嶇紪鐮佸櫒鍊间负100鍒欏綋鍓嶇紪鐮佸櫒鐩稿浣嶇疆涓?0锛屼篃灏辨槸鍦ㄥ師鐐逛笂
            Pan_Current_Angle_Machine = (double)Pan_Current_Data * Circle_Value / Pan_Encoder_Range;    //褰撳墠鐩稿鍊?*360/4096寰楀埌鐩稿瑙掑害
            if(Pan_Encoder_Direction)
            {
                Pan_Real_Angle = 360.000 - Pan_Current_Angle_Machine;
            }
            else
            {
                Pan_Real_Angle = Pan_Current_Angle_Machine;
            }
            //*******************************************************************
            //用户设定数据
            if(Pan_Real_Angle - Pan_Origin_User < 0)                                                    //
            {
                Pan_Current_Angle_User = Pan_Real_Angle - Pan_Origin_User + Circle_Value;
            }
            else
            {
                Pan_Current_Angle_User = Pan_Real_Angle - Pan_Origin_User;
            }
            //*******************************************************************
        }
        break;
#endif
//*******************************************************************************************
        case 3://IC-MU
        {
            Pan_Current_Data = (Pan_Encoder_Position_Current - Pan_Machine_Origin) & 0x3ffff;
            Pan_Current_Angle_Machine = (double)Pan_Current_Data * Circle_Value / Pan_Encoder_Range;
            if(Pan_Encoder_Direction)
            {
                Pan_Real_Angle = 360.000 - Pan_Current_Angle_Machine;
            }
            else
            {
                Pan_Real_Angle = Pan_Current_Angle_Machine;
            }
            if(Pan_Real_Angle - Pan_Origin_User < 0)
            {
                Pan_Current_Angle_User = Pan_Real_Angle - Pan_Origin_User + Circle_Value;
            }
            else
            {
                Pan_Current_Angle_User = Pan_Real_Angle - Pan_Origin_User;
            }
        }
        break;
//*******************************************************************************************
        case 4://MT6825
        {
            Pan_Current_Data = (Pan_Encoder_Position_Real - Pan_Machine_Origin) & 0x3ffff;
            Pan_Current_Angle_Machine = (double)Pan_Current_Data * Circle_Value / Pan_Encoder_Range;
            if(Pan_Encoder_Direction)
            {
                Pan_Real_Angle = 360.000 - Pan_Current_Angle_Machine;
            }
            else
            {
                Pan_Real_Angle = Pan_Current_Angle_Machine;
            }
            //Pan_Real_Angle = 360.000 - Pan_Current_Angle_Machine;
            if(Pan_Real_Angle - Pan_Origin_User < 0)
            {
                Pan_Current_Angle_User = Pan_Real_Angle - Pan_Origin_User + Circle_Value;
            }
            else
            {
                Pan_Current_Angle_User = Pan_Real_Angle - Pan_Origin_User;
            }
            if (Pan_User_Limit_Switch == Pan_User_Limit_Enable)
            {
                Pan_Soft_Limit_FLAG = Pan_Soft_Limit(Pan_Real_Angle,Pan_User_Limit_L,Pan_User_Limit_R);
            }
        }
        break;
        default:
            break;
        }
        switch(tilt_encoder_type)
        {
//*************************杩欓儴鍒嗕负淇话缂栫爜鍣ㄦ暟鎹鐞嗭紝瑙掑害杞崲绛?******************************************************************
        case 1://AS5311
        {
        } break;
//*******************************************************************************************
#ifndef FOC
        case 2://AS5600
        {
            //*******************************************************************
            //编码器出厂数据
            Tilt_Current_Data = (Tilt_Encoder_Data - Tilt_Machine_Origin & 0xffff) & 0xfff;
            Tilt_Current_Angle_Machine = (double)Tilt_Current_Data * Circle_Value / Tilt_Encoder_Range;//
#ifdef PT_LOAC_2215
            Tilt_Real_Angle = Tilt_Current_Angle_Machine;
#else
            Tilt_Real_Angle = 360.000 - Tilt_Current_Angle_Machine;
#endif
            //*******************************************************************
            //用户设定数据
            if(Tilt_Real_Angle - Tilt_Origin_User < 0)
            {
                TCAU = Tilt_Real_Angle - Tilt_Origin_User + Circle_Value;
            }
            else
            {
                TCAU = Tilt_Real_Angle - Tilt_Origin_User;
            }
            if(!Tilt_Encoder_Direction)     //20220105 zhangxuezhi澧炲姞
            {
                Tilt_Current_Angle_User = TCAU;
            }
            else
                //if(Tilt_Encoder_Direction == Encoder_Inversion)   //20220121zhangxuezhi
            {
                Tilt_Current_Angle_User = Circle_Value - TCAU;
            }
            //*******************************************************************
            //垂直限位判断
            if(Origin_Machine_Flag)
            {
                Tilt_Soft_Limit_FLAG = Tilt_Soft_Limit(Tilt_Current_Angle_Machine,Tilt_Limit_Up,Tilt_Limit_Down);
            }
        }
        break;
#endif
//*******************************************************************************************
        case 3://IC-MU
        {
            Tilt_Current_Data = (Tilt_Encoder_Position_Current - Tilt_Machine_Origin) & 0x3ffff;
            Tilt_Current_Angle_Machine = (double)Tilt_Current_Data * Circle_Value / Tilt_Encoder_Range;
#ifdef PT_EXF
            Tilt_Real_Angle = 360.000 - Tilt_Current_Angle_Machine;
#else
            Tilt_Real_Angle = Tilt_Current_Angle_Machine;
#endif
            if(Tilt_Real_Angle - Tilt_Origin_User < 0)
            {
                TCAU = Tilt_Real_Angle - Tilt_Origin_User + Circle_Value;
            }
            else
            {
                TCAU = Tilt_Real_Angle - Tilt_Origin_User;
            }
//              if(Tilt_Encoder_Direction == Encoder_Forward)
//              {
//                  Tilt_Current_Angle_User = TCAU;
//              }
            if(Tilt_Encoder_Direction)
            {
                Tilt_Current_Angle_User = Circle_Value - TCAU;
            }
            else
            {
                Tilt_Current_Angle_User = TCAU;
            }
            Tilt_Real_Angle = Tilt_Current_Angle_User;
            //*******************************************************************
            //垂直限位判断
            if(Origin_Machine_Flag)
            {
#ifdef PT_EXF
                Tilt_Soft_Limit_FLAG = Tilt_Soft_Limit(Tilt_Real_Angle,Tilt_Limit_Up,Tilt_Limit_Down);
#else
                Tilt_Soft_Limit_FLAG = Tilt_Soft_Limit(Tilt_Current_Angle_Machine,Tilt_Limit_Up,Tilt_Limit_Down);
#endif
            }
        }
        break;
        case 4://MT6825
        {
            Tilt_Current_Data = (Tilt_Encoder_Position_Real - Tilt_Machine_Origin) & 0x3ffff;
            Tilt_Current_Angle_Machine = (double)Tilt_Current_Data * Circle_Value / Tilt_Encoder_Range;
//              if(Tilt_Encoder_Direction == Encoder_Forward)
//              {
//                  Tilt_Real_Angle = Tilt_Current_Angle_Machine;
//              }
//              else
//              {
//                  Tilt_Real_Angle = 360.0 - Tilt_Current_Angle_Machine;
//              }
#ifdef PT_EXF
            Tilt_Real_Angle = 360.000 - Tilt_Current_Angle_Machine;
#else
            Tilt_Real_Angle = Tilt_Current_Angle_Machine;
#endif
            if(Tilt_Real_Angle - Tilt_Origin_User < 0)
            {
                TCAU = Tilt_Real_Angle - Tilt_Origin_User + Circle_Value;
            }
            else
            {
                TCAU = Tilt_Real_Angle - Tilt_Origin_User;
            }
//              if(Tilt_Encoder_Direction == Encoder_Forward)
//              {
//                  Tilt_Current_Angle_User = TCAU;
//              }
            if(Tilt_Encoder_Direction)
            {
                Tilt_Current_Angle_User = Circle_Value - TCAU;
            }
            else
            {
                Tilt_Current_Angle_User = TCAU;
            }
            //Tilt_Real_Angle = Tilt_Current_Angle_User;  //冯剑增加，20220927张学治去除
            //*******************************************************************
            //垂直限位判断
            if(Origin_Machine_Flag)
            {
                Tilt_Soft_Limit_FLAG = Tilt_Soft_Limit(Tilt_Current_Angle_Machine,Tilt_Limit_Up,Tilt_Limit_Down);
            }
        }
        break;
        default:
            break;
        }
        pan_angle_query = Filter_Choose(&pan_data, Pan_Current_Angle_User,PWM_P);
        tilt_angle_query = Filter_Choose(&tilt_data, Tilt_Current_Angle_User,PWM_T);
        Pan_Current_Angle_User = pan_angle_query;
        Tilt_Current_Angle_User = tilt_angle_query;
        //***********************************
        //速度采集
#ifndef FOC
#if defined (PT_LOAC_2215) || defined (PT_EXF)
        Tilt_A_Speed = (TIM4->CNT - 28080);
        Pan_A_Speed = (TIM5->CNT - 28080);
#else
        Tilt_A_Speed = (28080 - TIM4->CNT);
        Pan_A_Speed = (TIM5->CNT - 28080);
#endif
        Tilt_A_Speed *= Scale_Velocity_Tilt;
        Pan_A_Speed *= Scale_Velocity_Pan;
        TIM5->CNT = 28080; 
        TIM4->CNT = 28080;
#else
			  Tilt_A_Speed = tilt_data.speed;
        Pan_A_Speed = -pan_data.speed;	
#endif
        //***********************************
        //***********************************
        //调速输出
        Motor_Speed_Pan(Pan_Encoder_State,
                        Pan_Motion_Mode,
                        Pan_Target_Speed,
                        Pan_A_Speed,
                        Pan_Location_Speed,
                        &Place_InOut_Pan,
                        Pan_Target_Angle,
                        Pan_Real_Angle,
                        Pan_Real_Angle_Last,
                        Pan_Soft_Limit_FLAG);
        Pan_Real_Angle_Last = Pan_Real_Angle;
        if(Tilt_Location_Speed > Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt)
        {
            Tilt_Location_Speed = Tilt_Ultimate_MaxSpeed * Speed_Scale_Tilt;
        }
        Motor_Speed_Tilt(Tilt_Encoder_State,
                         Tilt_Motion_Mode,
                         Tilt_Target_Speed,
                         Tilt_A_Speed,
                         Tilt_Location_Speed,
                         &Place_InOut_Tilt,
                         Tilt_Target_Angle,
                         Tilt_Real_Angle,
                         Tilt_Real_Angle_Last,
                         Tilt_Soft_Limit_FLAG);
        Tilt_Real_Angle_Last =  Tilt_Real_Angle;
        //***********************************
        OSTimeDlyHMSM(0,0,0,Task_Encoder_Cycle_Time,OS_OPT_TIME_HMSM_STRICT,&err);
#ifdef Output
				printf("angle:%.2f,%.2f\r\n",pan_angle_query,tilt_angle_query);
#endif
    }
}
//***************************************************************************************************
u32 Pan_Encoder_Position(u8 type)
{
    u32 temp_p = 0;
    switch(type)
    {
    case 0x01:
        temp_p = Encoder_SSI_Pan();
        break;
#ifndef FOC
    case 0x02:
        temp_p = Pan_get_Angle();
        break;
#endif
    case 0x03:
        temp_p = Encoder_SPI_Pan(0xa6000000);
        break;
    case 0x04:
        temp_p = MT6825_Pan();
        break;
    default:
        temp_p = 0xeeee;
        break;
    }
    return  temp_p;
}
u32 Tilt_Encoder_Position(u8 type)
{
    u32 temp_t = 0;
    switch(type)
    {
    case 0x01:
        temp_t = Encoder_SSI_Tilt();
        break;
#ifndef FOC
    case 0x02:
        temp_t = Tilt_get_Angle();
        break;
#endif
    case 0x03:
        temp_t = Encoder_SPI_Tilt(0xa6000000);
        break;
    case 0x04:
        temp_t = MT6825_Tilt();
        break;
    default:
        temp_t = 0xeeee;
        break;
    }
#ifdef PT_HIAC_7280
    temp_t = 0x3ffff - temp_t;
#endif
    return temp_t;
}
u8 Pan_Soft_Limit(double Angle_User,double Limit_L,double Limit_R)
{
    u8 PanFLAG = 0;
    if (fabs(Limit_L - Limit_R) < 180)
    {
        if(Up_Or_Down(Angle_User,Limit_L) && Up_Or_Down(Limit_R,Angle_User))
        {
            if (Up_Or_Down((Limit_L + 2*Pan_Limit_Break > 360 ? Limit_L + 2*Pan_Limit_Break - 360 : Limit_L + 2*Pan_Limit_Break),Angle_User)&& Up_Or_Down(Limit_R,Angle_User))
				{
						PanFLAG = Pan_Limit_Near_L;
				}
				else if (Up_Or_Down(Angle_User,Limit_L) && Up_Or_Down(Angle_User,(Limit_R - 2*Pan_Limit_Break < 0 ? Limit_R - 2*Pan_Limit_Break + 360 :Limit_R - 2*Pan_Limit_Break)))
				{
						PanFLAG = Pan_Limit_Near_R;
				}
				else
				{
						PanFLAG = Pan_Limit_None;
				}
        }
        else
        {
            PanFLAG = Pan_Limit_Over;
        }
    }
    else
    {
        if (Angle_User > Limit_L && Angle_User < Limit_R)
        {
            if(Angle_User > Limit_L && Angle_User < Limit_L + Pan_Limit_Break *3)
            {
                PanFLAG = Pan_Limit_Near_L;
            }
            else if(Angle_User < Limit_R && Angle_User > Limit_R - Pan_Limit_Break *3)
            {
                PanFLAG = Pan_Limit_Near_R;
            }
            else
            {
                PanFLAG = Pan_Limit_None;
            }
        }
        else
        {
            PanFLAG = Pan_Limit_Over;
        }
    }
    return PanFLAG;
}

u8 Tilt_Soft_Limit(double Angle_User,double Limit_Up,double Limit_Down)
{
    u8 TiltFLAG = 0;
    if(Angle_User >= Limit_Up && Angle_User < 180)
    {
        TiltFLAG = Tilt_Limit_Over_Up;
        tilt_angle_query = Tilt_Angle_Limit_Up;
    }
    else if(Limit_Up - Angle_User <= Tilt_Limit_Brake && Angle_User < 180)
    {
        TiltFLAG = Tilt_Limit_Near_Up;
    }
    else if(Angle_User <= Limit_Down && Angle_User > 180)
    {
        TiltFLAG = Tilt_Limit_Over_Down;
        tilt_angle_query = Tilt_Angle_Limit_Down;
    }
    else if(Angle_User - Limit_Down <= Tilt_Limit_Brake && Angle_User > 180)
    {
        TiltFLAG = Tilt_Limit_Near_Down;
    }
    else
    {
        TiltFLAG = Tilt_Limit_None;
    }
    return TiltFLAG;
}
/*-------------------------------------------------------------------------------------------------------------*/
/*
        Q:杩囩▼鍣０锛孮澧炲ぇ锛屽姩鎬佸搷搴斿彉蹇紝鏀舵暃绋冲畾鎬у彉鍧?
        R:娴嬮噺鍣０锛孯澧炲ぇ锛屽姩鎬佸搷搴斿彉鎱紝鏀舵暃绋冲畾鎬у彉濂?
*/
static double p_last = 1;
double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction)
{
    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;
    double x_mid;
    double x_now;
    double p_mid;
    double p_now;
    double kg;
    x_mid = x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = p_last + Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=鍣０
    kg = p_mid / (p_mid + R); //kg涓簁alman filter锛孯涓哄櫔澹?
    x_now = x_mid + kg * (ResrcData - x_mid);//浼拌鍑虹殑鏈?浼樺??
    p_now = (1 - kg) * p_mid;//鏈?浼樺?煎搴旂殑covariance
    p_last = p_now; //鏇存柊covariance鍊?
    x_last = x_now; //鏇存柊绯荤粺鐘舵?佸??
    return x_now;
}
////**********************************************************
////鍙岀紦瀛樻护娉?
////BUF_A鎸夌収鍏堣繘鍏堝嚭鍘熷垯鍘嬫爤
////BUF_A       |   0   |   1   |   2   |   3   |   ......  |   100     |
////            |   a0  |   a1  |   a2  |   a3  |   ......  |   a100    |
////BUF_B鎸夌収鍒ゆ柇鏉′欢 濡傛灉|An - An-1| > 鍗曚綅閲囨牱鐨勭紪鐮佸櫒鍊煎垯杩涘叆涓嬩竴灞傚垽鏂紝濡傛灉|An - Bn| < 鍗曚綅閲囨牱鐨勭紪鐮佸櫒鍊煎垯灏咥n瀛樺偍鍒癇n+1鐩村埌
////**********************************************************
//void STMFLASH_Write_1(u32 WriteAddr,Encoder_Position_Store_t *data_t)
//{
//  uint32_t i;
//  uint32_t data_len;
//    FLASH_Unlock();
//    FLASH_DataCacheCmd(DISABLE);
//  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
//  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
//  //********************************************************************************************************************************************
//  //鍐檖an_data_a銆乸an_data_b銆乼ilt_data_a銆乼ilt_data_b闀垮害
//  data_len = 0;
//  data_len |= data_t->pan_data_a_len;
//  data_len <<= 8;
//  data_len |= data_t->pan_data_b_len;
//  data_len <<= 8;
//  data_len |= data_t->tilt_data_a_len;
//  data_len <<= 8;
//  data_len |= data_t->tilt_data_b_len;
//
//  FLASH_ProgramWord(data_buf_len_addr,data_len);
//  //鍐檖an_data_a鏁扮粍
//  for(i = 0;i < data_t->pan_data_a_len;i ++)
//  {
//      FLASH_ProgramWord(pan_data_a_addr + i * 4,data_t->pan_data_a[i]);
//  }
//  //鍐檖an_data_b鏁扮粍
//  for(i = 0;i < data_t->pan_data_b_len;i ++)
//  {
//      FLASH_ProgramWord(pan_data_b_addr + i * 4,data_t->pan_data_b[i]);
//  }
//  //鍐檛ilt_data_a鏁扮粍
//  for(i = 0;i < data_t->tilt_data_a_len;i ++)
//  {
//      FLASH_ProgramWord(tilt_data_a_addr + i * 4,data_t->tilt_data_a[i]);
//  }
//  //鍐檛ilt_data_b鏁扮粍
//  for(i = 0;i < data_t->tilt_data_b_len;i ++)
//  {
//      FLASH_ProgramWord(tilt_data_b_addr + i * 4,data_t->tilt_data_b[i]);
//  }
//  //鍐檖an_track_offset
//  FLASH_ProgramWord(pan_offset_addr,data_t->pan_offset_data);
//  //鍐檛ilt_track_offset
//  FLASH_ProgramWord(tilt_offset_addr,data_t->tilt_offset_data);
//
//  //鍐檕ffset_flag
//  FLASH_ProgramWord(offset_flag_addr,data_t->offset_flag);
//
//    FLASH_DataCacheCmd(ENABLE);
//    FLASH_Lock();
//}
//*********************************************************
//void Encoder_Init(void)
//{
//    Q_ERR err_q;
//    u8 i;
//    u32 buf_len = 0;
//    u8 p_a_l = 0;
//    u8 p_b_l = 0;
//    u8 t_a_l = 0;
//    u8 t_b_l = 0;
//    uint32_t offect_flag = 0;
//    u32 position_pan_a = 0,position_tilt_a = 0;
//    u32 position_pan_b = 0,position_tilt_b = 0;
//    u8 send_data[5]= {0};
//    //鍒涘缓缂撳瓨绌洪棿瀛樺偍缂栫爜鍣ㄥ??
//    //鍒涘缓A缂撳瓨涓篎IFO銆丅缂撳瓨涓洪攣瀹氬瓨鍌?
//    J_Create_Queue(&Encoder_Position_Pan_A,Encoder_Position_BUF_A_LEN,Encoder_Position_Pan_BUF_A,32,JQ_NO_LOCK_FIFO_QUEUE,&err_q);
//    J_Create_Queue(&Encoder_Position_Pan_B,Encoder_Position_BUF_B_LEN,Encoder_Position_Pan_BUF_B,32,JQ_LOCK_FIFO_QUEUE,&err_q);
//    J_Create_Queue(&Encoder_Position_Tilt_A,Encoder_Position_BUF_A_LEN,Encoder_Position_Tilt_BUF_A,32,JQ_NO_LOCK_FIFO_QUEUE,&err_q);
//    J_Create_Queue(&Encoder_Position_Tilt_B,Encoder_Position_BUF_B_LEN,Encoder_Position_Tilt_BUF_B,32,JQ_LOCK_FIFO_QUEUE,&err_q);
//    //璇诲嚭鍚勪釜缂撳瓨闀垮害
//    buf_len = STMFLASH_ReadWord(data_buf_len_addr);
//    if(buf_len != 0xffffffff)//BUF闀垮害涓嶄负绌?
//    {
//        p_a_l = buf_len >> 24;
//        p_b_l = buf_len >> 16;
//        t_a_l = buf_len >> 8;
//        t_b_l = buf_len & 0xff;
//        if(p_a_l)
//        {
//            //璇诲嚭Pan_BUF_A
//            for(i = 0; i < p_a_l; i  ++)
//            {
//                position_pan_a = STMFLASH_ReadWord(pan_data_a_addr + i * 4);
//                //鍐欏叆缂撳瓨A
//                J_Enqueue(&Encoder_Position_Pan_A,&position_pan_a,&err_q);
//            }
//        }
//        if(p_b_l)
//        {
//            //璇诲嚭Pan_BUF_B
//            for(i = 0; i < p_b_l; i ++)
//            {
//                position_pan_b = STMFLASH_ReadWord(pan_data_b_addr + i * 4);
//                //鍐欏叆缂撳瓨B
//                J_Enqueue(&Encoder_Position_Pan_B,&position_pan_b,&err_q);
//            }
//        }
//        if(t_a_l)
//        {
//            //璇诲嚭Tilt_BUF_A
//            for(i = 0; i < t_a_l; i ++)
//            {
//                position_tilt_a = STMFLASH_ReadWord(tilt_data_a_addr + i * 4);
//                //鍐欏叆缂撳瓨
//                J_Enqueue(&Encoder_Position_Tilt_A,&position_tilt_a,&err_q);
//            }
//        }
//        if(t_b_l)
//        {
//            //璇诲嚭Tilt_BUF_B
//            for(i = 0; i < t_b_l; i ++)
//            {
//                position_tilt_b = STMFLASH_ReadWord(tilt_data_b_addr + i * 4);
//                //鍐欏叆缂撳瓨B
//                J_Enqueue(&Encoder_Position_Tilt_B,&position_tilt_b,&err_q);
//            }
//        }
//        //璇诲嚭offset_flag
//        offect_flag = STMFLASH_ReadWord(offset_flag_addr);
//        if((offect_flag & 0x00010000) == 0x00010000)
//        {
//            //璇诲嚭pan_track_offset
//            pan_track_offset = STMFLASH_ReadWord(pan_offset_addr);
//        }
//        if((offect_flag & 0x00000001) == 0x00000001)
//        {
//            //璇诲嚭tilt_track_offset
//            tilt_track_offset = STMFLASH_ReadWord(tilt_offset_addr);
//        }
//    }
//    send_data[0] = 0xaa;
//    send_data[1] = pan_track_offset;
//    send_data[2] = 0xaa;
//    send_data[3] = tilt_track_offset;
//    send_data[4] = 0xaa;
//    Post_Information_Function_TX(Sem_USART_TX,&USART2_TX_BOX,2,send_data,5);
//}
//void Encoder_Position_Storage(void)
//{
//  Q_ERR err_q;
//  //鑾峰彇鍚勪釜缂撳瓨闀垮害
//  Encoder_Position_Store.pan_data_a_len = J_GetQueueDataLen(&Encoder_Position_Pan_A);
//  Encoder_Position_Store.pan_data_b_len = J_GetQueueDataLen(&Encoder_Position_Pan_B);
//  Encoder_Position_Store.tilt_data_a_len = J_GetQueueDataLen(&Encoder_Position_Tilt_A);
//  Encoder_Position_Store.tilt_data_b_len = J_GetQueueDataLen(&Encoder_Position_Tilt_B);
//  //鑾峰彇鍚勪釜缂撳瓨鏁版嵁
//  J_GetAllData(&Encoder_Position_Pan_A,Encoder_Position_Store.pan_data_a,&err_q);
//  J_GetAllData(&Encoder_Position_Pan_B,Encoder_Position_Store.pan_data_b,&err_q);
//  J_GetAllData(&Encoder_Position_Tilt_A,Encoder_Position_Store.tilt_data_a,&err_q);
//  J_GetAllData(&Encoder_Position_Tilt_B,Encoder_Position_Store.tilt_data_b,&err_q);
//  //鑾峰彇鍋忕Щ绯绘暟
//  if(pan_track_offset)
//  {
//      Encoder_Position_Store.offset_flag |= 0x00010000;
//      Encoder_Position_Store.pan_offset_data = pan_track_offset;
//  }
//  if(tilt_track_offset)
//  {
//      Encoder_Position_Store.offset_flag |= 0x00000001;
//      Encoder_Position_Store.tilt_offset_data = tilt_track_offset;
//  }
//
//  STMFLASH_Write_1(track_offect,&Encoder_Position_Store);
//}
