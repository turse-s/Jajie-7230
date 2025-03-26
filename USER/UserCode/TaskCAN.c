#include "TaskCAN.h"
#include "CRC16.h"
#include "stmflash.h"
#include "hardware.h"
//u8 updata_sum = 0;
FNC jumptoapp;
//u32 iapbuf[512];  //2K字节缓存
/*
通讯格式：
    Byte
        0:头H：0xff
        1:头L：0xd3
        2:有效数据长度(n)
        3:帧序号
        4:帧状态（反馈）
        5:
        ：
        ：   有效数据
        ：
        n + 5:CRC校验L
        n + 6:CRC校验H
*/
//u16 ID0_CRC = 0,ID1_CRC = 0,ID2_CRC = 0,ID_CRC_CRC = 0;
//void Task_CAN(void *p)
//{
//    OS_ERR err;
//    u32 Device_ID_0 = 0,Device_ID_1 = 0,Device_ID_2 = 0;
//    u8 Device_ID0[4] = {0};
//    u8 Device_ID1[4] = {0};
//    u8 Device_ID2[4] = {0};
//    u8 Device_ID_CRC[6] = {0};
//    Device_ID_0 = STMFLASH_ReadWord(0x1FFF7A10);
//    Device_ID_1 = STMFLASH_ReadWord(0x1FFF7A14);
//    Device_ID_2 = STMFLASH_ReadWord(0x1FFF7A18);
//    p = p;
////    TIM10_Init(TIM10_ARR,TIM10_PSC);//1ms
//    CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);
//    OSSemCreate ((OS_SEM*   )&Sem_CAN,
//                 (CPU_CHAR* )"Sem_CAN",
//                 (OS_SEM_CTR)0,
//                 (OS_ERR*   )&err);
//    Device_ID_0 = STMFLASH_ReadWord(0x1FFF7A10);
//    Device_ID_1 = STMFLASH_ReadWord(0x1FFF7A14);
//    Device_ID_2 = STMFLASH_ReadWord(0x1FFF7A18);
//    Device_ID0[0] = (Device_ID_0 >> 24) & 0xff;
//    Device_ID0[1] = (Device_ID_0 >> 16) & 0xff;
//    Device_ID0[2] = (Device_ID_0 >> 8) & 0xff;
//    Device_ID0[3] = Device_ID_0 & 0xff;
//    ID0_CRC = CRC_16(Device_ID0,4);
//    Device_ID1[0] = (Device_ID_1 >> 24) & 0xff;
//    Device_ID1[1] = (Device_ID_1 >> 16) & 0xff;
//    Device_ID1[2] = (Device_ID_1 >> 8) & 0xff;
//    Device_ID1[3] = Device_ID_1 & 0xff;
//    ID1_CRC = CRC_16(Device_ID1,4);
//    Device_ID2[0] = (Device_ID_2 >> 24) & 0xff;
//    Device_ID2[1] = (Device_ID_2 >> 16) & 0xff;
//    Device_ID2[2] = (Device_ID_2 >> 8) & 0xff;
//    Device_ID2[3] = Device_ID_2 & 0xff;
//    ID2_CRC = CRC_16(Device_ID2,4);
//    Device_ID_CRC[0] = ID0_CRC & 0xff;
//    Device_ID_CRC[1] = ID0_CRC >> 8 & 0xff;
//    Device_ID_CRC[2] = ID1_CRC & 0xff;
//    Device_ID_CRC[3] = ID1_CRC >> 8 & 0xff;
//    Device_ID_CRC[4] = ID2_CRC & 0xff;
//    Device_ID_CRC[5] = ID2_CRC >> 8 & 0xff;
//    ID_CRC_CRC = CRC_16(Device_ID_CRC,6);
//    Assigned_Address_Flag = Assigned_Address;
//    while(1)
//    {
//        if(Assigned_Address_Flag == Unassigned_Address)
//        {
//            Data_PostBack(Assign_Address);
//            OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
//        }
//        else
//        {
//            OSSemPend(&Sem_CAN,0,OS_OPT_PEND_BLOCKING,0,&err);
//            Pend_Information_Function(&CAN_BOX,CAN_Function);
//        }
//    }
//}
//u8 Frame_HAT_LEN = 5;
//u16 Frame_LEN;
//u16 Frame_NUM;
//u16 Frame_STA;
//u8 Frame_Data[300];
//u16 Frame_CAN_num = 0;
//u16 Frame_Data_num = 0;
//u16 check_data;
//u8 check_buf[270];
//void CAN1_RX_Data(u8 *data_buf,u8 len)
//{
//    u8 i = 0;
//    for(i = 0; i < len; i ++)
//    {
//        Frame_Data[Frame_Data_num * 8 + i] = data_buf[i];
//        Frame_CAN_num ++;
//    }
//    if(Assigned_Address_Flag == Unassigned_Address)//未分配本机设备通讯通道
//    {
//        Frame_CAN_num = 0;
//        if(((u16)Frame_Data[0] << 8) + Frame_Data[1] == ID0_CRC
//                && ((u16)Frame_Data[2] << 8) + Frame_Data[3] == ID1_CRC
//                && ((u16)Frame_Data[4] << 8) + Frame_Data[5] == ID2_CRC)
//        {
//            Local_CAN_StdId = ((u16)Frame_Data[6] << 8) + Frame_Data[7];
//            CAN1_ID_Init(Local_CAN_StdId);
//            Assigned_Address_Flag = Assigned_Address;
//            Data_PostBack(Succeed);
//        }
//        else
//        {
//            Data_PostBack(Failure);
//        }
//    }
//    else//已分配本机设备通讯通道
//    {
//        if(Frame_Data_num == 0)
//        {
//            if(Frame_Data[0] == 0xff && Frame_Data[1] == 0xd3)
//            {
//                Frame_LEN = Frame_Data[2];
//                Frame_NUM = Frame_Data[3];
//                Frame_STA = Frame_Data[4];
//                Frame_Data_num = 1;
//            }
//            else
//            {
//                Frame_Data_num = 0;
//                Frame_CAN_num = 0;
//            }
//        }
//        else
//        {
//            if(Frame_CAN_num == (Frame_LEN + 7))
//            {
//                check_data = CRC_16(Frame_Data,Frame_LEN + 5);
//                if(check_data == (((Frame_Data[Frame_LEN + 6] | 0x0000) << 8) + Frame_Data[Frame_LEN + 5]))
//                {
//                    Post_Information_Function_CAN(Sem_CAN,&CAN_BOX,Frame_Data,Frame_HAT_LEN,Frame_LEN + Frame_HAT_LEN);
//                }
//                check_data = 0;
//                Frame_Data_num = 0;
//                Frame_CAN_num = 0;
//            }
//            else
//            {
//                Frame_Data_num ++;
//            }
//        }
//    }
//}
//char Command_Type[] = {"type:"};
//char Command_Type_Stop[] = {"stop"};
//char Command_Type_Run[] = {"run"};
//char Command_Type_Goto[] = {"goto"};
//char Command_Type_Query_Angle[] = {"query_angle"};
//char Command_Type_Query_Versions[] = {"query_versions"};
//char Command_Type_Zero_Set[] = {"zero"};
//char Command_Type_Upgrade[] = {"upgrade"};
//char Command_Type_Pan_PID[] = {"pan_pid"};
//char Command_Type_Tilt_PID[] = {"tilt_pid"};
//char Data_Type_P_S[] = {"P_S:"};
//char Data_Type_T_S[] = {"T_S:"};
//char Data_Type_P_A[] = {"pan_angle:"};
//char Data_Type_T_A[] = {"tilt_angle:"};
//char Data_Type_Result[] = {"result:"};
//char Data_Type_Kp[] = {"kp:"};
//char Data_Type_Ki[] = {"ki:"};
//char Data_Type_Kd[] = {"kd:"};
//char Data_Type_Version[] = {"version,"};
//char PostBack_Type_Angle[] = {"angle,"};
//char PostBack_Type_State[] = {"state,"};
//char PostBack_Type_State_Succeed[] = {"succeed,"};
//char PostBack_Type_State_Failure[] = {"failure,"};
//void CAN_Function(void)
//{
//    u8 i = 0,j = 0,k = 0;
//    u8 Section_Number1 = 0,Section_Number2 = 0;
//    u8 Section_Title_Len = 0,Section_Detail_Len = 0;
//    char Section_Title[255] = {0};
//    for(i = 0; i < Frame_LEN + Frame_HAT_LEN; i ++)
//    {
//        char Section_Detail[255] = {0};
//        if(CAN_BOX.Memory_Data_1[CAN_BOX.extract_finger][i] == ':')
//        {
//            Section_Number1 = i;
//            for(j = Section_Number2; j < i; j ++)
//            {
//                Section_Title[j - Section_Number2] = CAN_BOX.Memory_Data_1[CAN_BOX.extract_finger][j];
//            }
//            Section_Title_Len = Section_Number1 -  Section_Number2;
//        }
//        if(CAN_BOX.Memory_Data_1[CAN_BOX.extract_finger][i] == ',')
//        {
//            Section_Number2 = i + 1;
//            for(k = Section_Number1 + 1; k < i; k ++)
//            {
//                Section_Detail[k - Section_Number1 - 1] = CAN_BOX.Memory_Data_1[CAN_BOX.extract_finger][k];
//            }
//            Section_Detail_Len = Section_Number2 - Section_Number1 - 2;
//            //*************************************************************************************
//            Valid_Data(Section_Title,Section_Detail,Section_Title_Len,Section_Detail_Len);
//            //*************************************************************************************
//        }
//    }
//}
//u16 pid_flag = 0;
//void Valid_Data(char *title,char *detail,u8 title_num,u8 detail_num)
//{
//    float PTS = 0,TTS = 0;
//    u32 data[1] = {0};
//    if(!memcmp(title,Command_Type,title_num))
//    {
//        if(!memcmp(detail,Command_Type_Stop,detail_num))//停止
//        {
//            Pan_Motion_Mode = Rocker_Motion;
//            Tilt_Motion_Mode = Rocker_Motion;
//            Pan_Target_Speed = 0;
//            Tilt_Target_Speed = 0;
//            Special_Function = Tour_Scan_Disable;
//            Stay_Time = 0;
//        }
//        else if(!memcmp(detail,Command_Type_Run,detail_num))//摇杆动作
//        {
//            Pan_Motion_Mode = Rocker_Motion;
//            Tilt_Motion_Mode = Rocker_Motion;
//        }
//        else if(!memcmp(detail,Command_Type_Goto,detail_num))//指定角度
//        {
//            Pan_Motion_Mode = Location_Motion;
//            Tilt_Motion_Mode = Location_Motion;
//        }
//        else if(!memcmp(detail,Command_Type_Query_Angle,detail_num))//查询角度
//        {
//            Data_PostBack(Query_Angle);
//        }
//        else if(!memcmp(detail,Command_Type_Query_Versions,detail_num))//查询版本
//        {
//            Data_PostBack(Query_Version);
//        }
//        else if(!memcmp(detail,Command_Type_Zero_Set,detail_num))//设定零点
//        {
//            Pan_Origin_User = Pan_Real_Angle;
//            Tilt_Origin_User = Tilt_Real_Angle;
//        }
//        else if(!memcmp(detail,Command_Type_Upgrade,detail_num))//固件升级
//        {
//            data[0] = upgrade + Local_CAN_StdId;
//            APP_GOTO_IAP(Upgrade_CAN_ID,data,1);
//        }
//        else if(!memcmp(detail,Command_Type_Pan_PID,detail_num))//设置PID参数
//        {
//            pid_flag = 1;
//        }
//        else if(!memcmp(detail,Command_Type_Tilt_PID,detail_num))//设置PID参数
//        {
//            pid_flag = 2;
//        }
//    }
//    else if(!memcmp(title,Data_Type_P_S,title_num))
//    {
//        if(Pan_Motion_Mode == Rocker_Motion)
//        {
//            PTS = atof(detail);
//            PTS = PTS * 380;
//            if(PTS > 5000)
//            {
//                PTS = 5000;
//            }
//            if(PTS < -5000)
//            {
//                PTS = -5000;
//            }
//            Pan_Target_Speed = PTS;
//        }
//        if(Pan_Motion_Mode == Location_Motion)
//        {
//            PTS = atof(detail);
//            PTS = PTS * 380;
//            if(PTS > 5000)
//            {
//                PTS = 5000;
//            }
//            Pan_Location_Speed = PTS;
//        }
//        Special_Function = Tour_Scan_Disable;
//        Stay_Time = 0;
//    }
//    else if(!memcmp(title,Data_Type_T_S,title_num))
//    {
//        if(Tilt_Motion_Mode == Rocker_Motion)
//        {
//            TTS = atof(detail);
//            TTS = TTS * 380;
//            if(TTS > 4000)
//            {
//                TTS = 4000;
//            }
//            if(TTS < -4000)
//            {
//                TTS = -4000;
//            }
//            Tilt_Target_Speed = TTS;
//        }
//        if(Tilt_Motion_Mode == Location_Motion)
//        {
//            TTS = atof(detail);
//            TTS = TTS * 380;
//            if(TTS > 4000)
//            {
//                TTS = 4000;
//            }
//            Tilt_Location_Speed = TTS;
//        }
//        Special_Function = Tour_Scan_Disable;
//        Stay_Time = 0;
//    }
//    else if(!memcmp(title,Data_Type_P_A,title_num))
//    {
//        Place_InOut_Pan = OutPlace;
//        Pan_Target_Angle = atof(detail);
//        Special_Function = Tour_Scan_Disable;
//        Stay_Time = 0;
//    }
//    else if(!memcmp(title,Data_Type_T_A,title_num))
//    {
//        Place_InOut_Tilt = OutPlace;
//        Tilt_Target_Angle = atof(detail);
//        Special_Function = Tour_Scan_Disable;
//        Stay_Time = 0;
//    }
//    else if(!memcmp(title,Data_Type_Kp,title_num))
//    {
//        if(pid_flag == 1)
//        {
//            Pan_Kp = atof(detail);
//        }
//        if(pid_flag == 2)
//        {
//            Tilt_Kp = atof(detail);
//        }
//    }
//    else if(!memcmp(title,Data_Type_Ki,title_num))
//    {
//        if(pid_flag == 1)
//        {
//            Pan_Ki = atof(detail);
//        }
//        if(pid_flag == 2)
//        {
//            Tilt_Ki = atof(detail);
//        }
//    }
//    else if(!memcmp(title,Data_Type_Kd,title_num))
//    {
//        if(pid_flag == 1)
//        {
//            Pan_Kd = atof(detail);
//        }
//        if(pid_flag == 2)
//        {
//            Tilt_Kd = atof(detail);
//        }
//    }
//}
////******************************************************************************************
////
//void Data_PostBack(u16 PostBack_Type)
//{
//    char PostBack_Frame[300] = {0};
//    char Data[7] = {0};
//    char comma[1] = {","};
//    u16 check;
//    char check_data[2];
//    u8 len;
//    switch(PostBack_Type)
//    {
//    case Succeed:
//    {
//        PostBack_Frame[0] = 0xff;
//        PostBack_Frame[1] = 0xd3;
//        PostBack_Frame[2] = 0x05;
//        PostBack_Frame[3] = 0x01;
//        PostBack_Frame[4] = 0x01;
//        strcat(PostBack_Frame,Command_Type);
//        strcat(PostBack_Frame,PostBack_Type_State);
//        strcat(PostBack_Frame,Data_Type_Result);
//        strcat(PostBack_Frame,PostBack_Type_State_Succeed);
//        len = My_strlen(PostBack_Frame);
//        PostBack_Frame[2] = len - 5;
//        check = CRC_16((const u8 *)PostBack_Frame,len);
//        check_data[0] = check & 0xff;
//        check_data[1] = check >> 8;
//        strcat(PostBack_Frame,check_data);
//        PostBack_Founction(PostBack_Frame,len + 2);//发送
//    }
//    break;
//    case Failure:
//    {
//        PostBack_Frame[0] = 0xff;
//        PostBack_Frame[1] = 0xd3;
//        PostBack_Frame[2] = 0x05;
//        PostBack_Frame[3] = 0x01;
//        PostBack_Frame[4] = 0x01;
//        strcat(PostBack_Frame,Command_Type);
//        strcat(PostBack_Frame,PostBack_Type_State);
//        strcat(PostBack_Frame,Data_Type_Result);
//        strcat(PostBack_Frame,PostBack_Type_State_Failure);
//        len = My_strlen(PostBack_Frame);
//        PostBack_Frame[2] = len - 5;
//        check = CRC_16((const u8 *)PostBack_Frame,len);
//        check_data[0] = check & 0xff;
//        check_data[1] = check >> 8;
//        strcat(PostBack_Frame,check_data);
//        PostBack_Founction(PostBack_Frame,len + 2);//发送
//    }
//    break;
//    case Assign_Address:
//    {
//        PostBack_Frame[0] = (ID0_CRC >> 8) & 0xff;
//        PostBack_Frame[1] = ID0_CRC & 0xff;
//        PostBack_Frame[2] = (ID1_CRC >> 8) & 0xff;
//        PostBack_Frame[3] = ID1_CRC & 0xff;
//        PostBack_Frame[4] = (ID2_CRC >> 8) & 0xff;
//        PostBack_Frame[5] = ID2_CRC & 0xff;
//        PostBack_Frame[6] = ID_CRC_CRC & 0xff;
//        PostBack_Frame[7] = (ID_CRC_CRC >> 8) & 0xff;
//        Local_CAN_StdId = 0x0001;
//        PostBack_Founction(PostBack_Frame,8);
//    }
//    break;
//    case Query_Angle:
//    {
//        PostBack_Frame[0] = 0xff;
//        PostBack_Frame[1] = 0xd3;
//        PostBack_Frame[2] = 0x05;
//        PostBack_Frame[3] = 0x01;
//        PostBack_Frame[4] = 0x01;
//        strcat(PostBack_Frame,Command_Type);
//        strcat(PostBack_Frame,PostBack_Type_Angle);
//        strcat(PostBack_Frame,Data_Type_P_A);
//        sprintf(Data,"%3.3f",Pan_Real_Angle);
//        strcat(PostBack_Frame,Data);
//        strcat(PostBack_Frame,comma);
//        strcat(PostBack_Frame,Data_Type_T_A);
//        sprintf(Data,"%3.3f",Tilt_Current_Angle_User);
//        strcat(PostBack_Frame,Data);
//        strcat(PostBack_Frame,comma);
//        len = My_strlen(PostBack_Frame);
//        PostBack_Frame[2] = len - 5;
//        check = CRC_16((const u8 *)PostBack_Frame,len);
//        check_data[0] = check & 0xff;
//        check_data[1] = check >> 8;
//        strcat(PostBack_Frame,check_data);
//        PostBack_Founction(PostBack_Frame,len + 2);//发送
//    }
//    break;
////        case Query_Version:
////        {
////            PostBack_Frame[0] = 0xff;
////            PostBack_Frame[1] = 0xd3;
////            PostBack_Frame[2] = 0x05;
////            PostBack_Frame[3] = 0x01;
////            PostBack_Frame[4] = 0x01;
////
////            strcat(PostBack_Frame,Command_Type);
////            strcat(PostBack_Frame,Command_Type_Query_Versions);
////
////            strcat(PostBack_Frame,Data_Type_Version);
////            strcat(PostBack_Frame,PT_Version);
////            strcat(PostBack_Frame,comma);
////
////            len = My_strlen(PostBack_Frame);
////            PostBack_Frame[2] = len - 5;
////            check = CRC_16((const u8 *)PostBack_Frame,len);
////            check_data[0] = check & 0xff;
////            check_data[1] = check >> 8;
////            strcat(PostBack_Frame,check_data);
////
////            PostBack_Founction(PostBack_Frame,len + 2);//发送
////        }break;
//    default:
//        break;
//    }
//}
//u8 PostBack_Founction(char *data,u8 len)
//{
//    u8 state = 0,i = 0;
//    for(i = 0; i < len / 8; i ++)
//    {
//        state = CAN1_Send_Msg(Local_CAN_StdId,Local_CAN_ExtId,Local_CAN_IDE,Local_CAN_RTR,8,data,i * 8);
//    }
//    state = CAN1_Send_Msg(Local_CAN_StdId,Local_CAN_ExtId,Local_CAN_IDE,Local_CAN_RTR,len % 8,data,len - len % 8);
//    return state;
//}
//void PostBack_USART(u8 *data)
//{
//    char data_u[14];
//    u16 chk = 0;
//    data_u[0] = 0xff;
//    data_u[1] = 0xd3;
//    data_u[2] = 7;
//    data_u[3] = 0x01;
//    data_u[4] = 0x01;
//    data_u[5] = data[0];
//    data_u[6] = data[1];
//    data_u[7] = data[2];
//    data_u[8] = data[3];
//    data_u[9] = data[4];
//    data_u[10] = data[5];
//    data_u[11] = data[6];
//    chk = CRC_16((const u8 *)data_u,14);
//    data_u[12] = chk & 0xff;
//    data_u[13] = chk >> 8;
//    PostBack_Founction(data_u,14);
//}
//void SoftReset(void)
//{
//    __set_FAULTMASK(1); // 关闭所有中断
//    NVIC_SystemReset(); // 复位
//}
u8 mbox;
u8 CAN1_Send_Msg(u32 CAN_StdId,u32 CAN_ExtId,u8 CAN_IDE,u8 CAN_RTR,u8 CAN_DLC,u8 *msg,u8 start)
{
    u16 i=0;
    TxMessage.StdId = CAN_StdId;    //标准标识符
    TxMessage.ExtId = CAN_ExtId;    //扩展标示符
    TxMessage.IDE = CAN_IDE;        //
    TxMessage.RTR = CAN_RTR;        //
    TxMessage.DLC = CAN_DLC;        //发送字节数
    for(i = 0; i < CAN_DLC; i ++)
    {
        TxMessage.Data[i]=msg[i + start];
    }   // 第一帧信息
    mbox = CAN_Transmit(CAN1,&TxMessage);
    i=0;
    while((CAN_TransmitStatus(CAN1,mbox) != CAN_TxStatus_Ok)&&(i<0XFFF))
    {
        i++;    //等待发送结束
    }
    if(i>=0XFFF)
    {
        return 1;
    }
    return 0;
}
void iap_load_app(u32 appxaddr)
{
//    __set_PRIMASK(1);
    if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000) //检查栈顶地址是否合法.
    {
//        RCC_DeInit();
//        INTX_DISABLE();
//        __set_FAULTMASK(1);
//        __disable_irq(); //关闭总中断
//        CAN_ITConfig(CAN1,CAN_IT_FMP0,DISABLE);
//        SysTick->CTRL |= ~SysTick_CTRL_TICKINT_Msk;    //开启SYSTICK中断
//        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
//        NVIC_SetVectorTable(NVIC_VectTab_FLASH,appxaddr);
        jumptoapp = (FNC)*(volatile unsigned int*)(appxaddr + 4);     //用户代码区第二个字为程序开始地址(复位地址)
        __set_PSP(*(volatile unsigned int*) appxaddr);              //重新设置进程PSP堆栈地址
        __set_CONTROL(0);                                           //设置为MSP
        __set_MSP(*(volatile unsigned int*) appxaddr);              //重新设置进程MSP堆栈地址
        NVIC_SetVectorTable(NVIC_VectTab_FLASH,appxaddr);
        jumptoapp();                                                //跳转到APP.
    }
}
void APP_GOTO_IAP(u32 upgrade_flag_addr,u32 *upgrade_flag_data,u32 upgrade_flag_data_len)
{
    STMFLASH_Write(upgrade_flag_addr,upgrade_flag_data,upgrade_flag_data_len);//记录当前CAN信道和升级标识
    Delay_xMS(100);
    SoftReset();//直接复位
}
