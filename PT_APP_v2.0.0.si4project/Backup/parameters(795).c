//**************************************
// File      : bsp.c
// Version   : V1.3.0
// By        : YangJing
// For       : PT_HIAC
// Device    : STM32F407VET6
//**************************************
#include "parameters.h"
#include "hardware.h"
u8 arr1 = 0;
//************************************
//系统状态记录
//**在flash地址0x08010000记录系统状态
//**数据为u32数据，4个字节byte3 byte2 byte1 byte0
//**byte3:记录程序编译状态，即未运行状态，记为0x10
//  byte2:记录程序升级状态，等待记为0x11，升级完成进入新程序记为0x10
//  byte1、byte0:记录CAN通信地址
//
//
//************************************
//const unsigned int SYSTEM_STATE_DATA __attribute__((at(0x08010000))) = 0x00000000;
//***********************************************************************************
//
//                                  Parameters
//
//***********************************************************************************
 //用于调整编码器输出值的方向 可以取Encoder_Forward或者Encoder_Inversion
#ifdef PT_HIAC_7280
u8 Tilt_Encoder_Direction = Encoder_Inversion;
u8 Pan_Encoder_Direction = Encoder_Forward;
#else
u8 Tilt_Encoder_Direction = Encoder_Forward;
u8 Pan_Encoder_Direction = Encoder_Inversion;
#endif
u8 PT_ID = 0x01;
//总线通讯
u8 Assigned_Address_Flag = Unassigned_Address;
u16 Local_CAN_StdId = 0x0001;//标准标识符
u32 Local_CAN_ExtId = 0x00000000;//扩展标识符
u8 Local_CAN_IDE = CAN_ID_STD;  //标准标识符
u8 Local_CAN_RTR = CAN_RTR_Data;//数据帧
u8 Local_CAN_DLC = 0x08;
u8 Local_CAN_BUF[8];
//**********************************************************************
u32 USART1_BAND;
u32 USART2_BAND;
u32 USART3_BAND;
u8 Axial = 0;///透传标识//调整护照支架
u16 Stay_Time;
u8 Delete_Preset_FLag = 0;
u16 Delete_Preset_Time = 0;
u8 Delete_Data_FLag = 0;
u16 Delete_Data_Time = 0;
//USART1
u8 USART1_RX_Byte = 0x00;
u8 USART1_RX_Byte_Num = 0;
u8 DMA_USART1_RX_Buf[DMA_USART1_RX_BUF_SIZE];
u8 DMA_USART1_TX_Buf[DMA_USART1_TX_BUF_SIZE];
u8 USART1_RX_Buf[DMA_USART1_RX_BUF_SIZE];
u8 USART1_Use_DMA_Tx_Flag = 0;
u16 USART1_data_pointer = 0;
u16 USART1_dispose_data_pointer = 0;
u8 USART1_First_Frame_Flag = 0;
u8 Angle_Accuracy_Flag = 0;//当前角度精度标志，0为普通精度；1为高精度
//USART2
u8 USART2_RX_Byte = 0x00;
u8 USART2_RX_Byte_Num = 0;
u8 DMA_USART2_RX_Buf[DMA_USART2_RX_BUF_SIZE];
u8 DMA_USART2_TX_Buf[DMA_USART2_TX_BUF_SIZE];
u8 USART2_Use_DMA_Tx_Flag = 0;
u8 USART2_RX_Buf[DMA_USART2_RX_BUF_SIZE];
u16 USART2_data_pointer = 0;
u16 USART2_dispose_data_pointer = 0;
//USART3
u8 USART3_RX_Byte = 0x00;
u8 USART3_RX_Byte_Num = 0;
u8 DMA_USART3_RX_Buf[DMA_USART3_RX_BUF_SIZE];
u8 DMA_USART3_TX_Buf[DMA_USART3_TX_BUF_SIZE];
u8 USART3_Use_DMA_Tx_Flag = 0;
u8 USART3_RX_Buf[DMA_USART3_RX_BUF_SIZE];
u16 USART3_data_pointer = 0;
u16 USART3_dispose_data_pointer = 0;
u8 USART3_Protocol_Type = Protocol_PELCO_D;
//u8 USART3_Protocol_Type = Protocol_VISCA;
u8 Empty_Buf[10] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u16 VISCA_CAM_Zoom_Flag = 0;
//USART_TX
u8 USART_TX_BUF[USART_TX_BUF_SIZE];
//Preset
u16 Zoom_Position = 0;
u16 Zoom_Position_Max = 0;
u16 Zoom_Position_Min = 0;
u16 Focus_Position = 0;
u16 Versions_Lens;
//u32 Preset_Data_Zoom[128];
u8 Preset_Extract_Flag = 0;
u8 Preset_Extract_num = 0;;
u16 Preset_Num[128];
u8 Preset_Number = 0;
u8 Preset_Empty = 0;
u8 Return_Mode = Return_Query;
u8 Return_Data_Type = Return_Angle;
u8 Self_Adaption = Self_Adaption_Enable;//恩博默认关闭自适应
u8 Special_Function = Tour_Scan_Disable;
u8 Watcher_Switch = OFF;
u8 watcher_pause_state = watcher_continue;//守望位暂停状态改为‘继续’
u8 Tour_Num = 0;
u8 Scan_Num = 0;
u16 Tour_Time = Tour_WaitTime_Min;//巡检预置位停留时间默认
u16 Scan_Time_Left = Scan_WaitTime_Min;//巡检预置位停留时间默认
u16 Scan_Time_Right = Scan_WaitTime_Min;//巡检预置位停留时间默认
u16 Scan_Zoom_Left = 0;
u16 Scan_Zoom_Right = 0;
u16 Watcher_Time = Watcher_WaitTime_Mid;
u16 Tour_Speed_Pan = Tour_Speed_Mid;
u16 Tour_Speed_Tilt = Tour_Speed_Mid;
u16 Scan_Speed = Scan_Speed_Mid;
u16 Watcher_Speed = Watcher_Speed_Mid;
double Watcher_Angle_Pan;
double Watcher_Angle_Tilt;
double Section_Border1_Angle;
double Section_Border2_Angle;
//Encoder
u8 Pan_Encoder_State = Encoder_Normal;
u8 Tilt_Encoder_State = Encoder_Normal;
double Angle_Decimal = 100.0;
u32 encoder_counter = 0;
double Pan_Encoder_Range = 4096;
double Pan_Angle_Accuracy = 0.1;
double Pan_Origin_User = 0;
double Pan_Real_Angle = 0;
double Pan_Real_Angle_1 = 0;
double Pan_Real_Angle_Last = 0;
double Pan_Current_Angle_User = 0;
double Pan_Target_Angle = 0;
double Pan_Current_Angle_Machine = 0;
double Pan_Machine_Origin_Angle = 0;
double Tilt_Encoder_Range = 4096;
double Tilt_Angle_Accuracy = 0.1;
double Tilt_Origin_User = 0;
double Tilt_Real_Angle = 0;
double Tilt_Real_Angle_1 = 0;
double Tilt_Real_Angle_Last = 0;
double Tilt_Current_Angle_User = 0;
double Tilt_Target_Angle = 0;
double Tilt_Target_Angle_Last = 0;
double Tilt_Current_Angle_Machine = 0;
double Tilt_Machine_Origin_Angle = 0;
Filter_DisappearsShakes_t Pan_Angle_Filter;
Filter_DisappearsShakes_t Tilt_Angle_Filter;
u16 pan_encoder_type = 0;
u16 tilt_encoder_type = 0;
double pan_angle_query = 0;
double tilt_angle_query = 0;
u8 U2_switch = 1;
u8 Pan_Init_Flag = 0;
u8 Pan_Section_Counter = 0;
u8 Pan_Section_Counter_Check = 0;
u32 Pan_Current_Data = 0;//0x80000
u16 Pan_Encoder_Data = 0;//
u32 Pan_Encoder_Data_Last = 0;//上次编码器值
u32 Pan_Machine_Origin;
u32 Pan_Encoder_Position_Current = 0;
u32 Pan_Encoder_Position_Save = 0;
u32 Pan_Encoder_Position_Real = 0;
u32 Tilt_Encoder_Position_Real = 0;
int pan_track_offset = 0;   //ICMU的补偿的偏移系数
int tilt_track_offset = 0;
int pan_track_err = 0;
int tilt_track_err = 0;
int Leading_mod=0;
u8 Tilt_Init_Flag = 0;
u8 Tilt_Section_Counter = 0;
u8 Tilt_Section_Counter_Check = 0;
u32 Tilt_Current_Data = 0;//0x80000
u32 Tilt_Encoder_Position_Save = 0;
u16 Tilt_Encoder_Data = 0;//
u32 Tilt_Encoder_Data_Last = 0;//上次编码器值
u32 Tilt_Machine_Origin;
u32 Tilt_Encoder_Position_Current = 0;
double Pan_User_Limit_L = 0;
double Pan_User_Limit_R = 0;
u8 Pan_User_Limit_Switch = 0;
u8 Pan_Soft_Limit_FLAG = Pan_Limit_None;
double Tilt_Limit_Up = Tilt_Angle_Limit_Up;
double Tilt_Limit_Down = Tilt_Angle_Limit_Down;
u8 Tilt_Hard_Limit_FLAG = Tilt_Limit_None;
u8 Tilt_Soft_Limit_FLAG = Tilt_Limit_None;
u8 Origin_Machine_Flag = 0;
u8 Origin_Machine_Record[8] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#ifdef ZS
u16 Extract_Number = 0;
#endif
#ifdef SH
Scan_Data Sweep[5] = {0,0,Tour_Speed_Mid,0,0};
#endif
//*******************************************************
//澧炲姞缂栫爜鍣ㄥ弻缂撳啿鏁扮粍锛岃褰曡烦鍖轰互琛ュ伩鍋忓樊
//Encoder_Position_BUF_t Encoder_Position_BUF_Pan = {.Position_Sampling_Period = Pan_MAX_Sampling_Period_Position};
//Encoder_Position_BUF_t Encoder_Position_BUF_Tilt = {.Position_Sampling_Period = Tilt_MAX_Sampling_Period_Position};
//*******************************************************
//Speed
int Pan_A_MaxSpeed = A_MaxSpeed;
int Tilt_A_MaxSpeed = A_MaxSpeed;
int Pan_A_MinSpeed = A_MinSpeed;
int Tilt_A_MinSpeed = A_MinSpeed;
int Pan_Ultimate_MaxSpeed;
int Tilt_Ultimate_MaxSpeed;
int Pan_Ultimate_MinSpeed;
int Tilt_Ultimate_MinSpeed;
float Speed_Scale_Pan = 1.0;
float Speed_Scale_Tilt = 1.0;
float Scale_Velocity_Pan = 1;
float Scale_Velocity_Tilt = 1;
u8 Speed_Set_Satus = 0;
//Motor
u16 Pan_Motion_Mode = Rocker_Motion;
u16 Tilt_Motion_Mode = Rocker_Motion;
u8 Place_InOut_Pan = OutPlace;
u8 Place_InOut_Tilt = OutPlace;
u8 Pan_Direction_Target = Direction_Stop;
double Pan_SetAngle_RetardSection = 40;
double Tilt_SetAngle_RetardSection = 20;
float ratio_add = 2;
float ratio_subtract_pan = 4;
float ratio_subtract_tilt = 4;
//double ratio_idle_pan = 2;
//double ratio_idle_tilt = 2;
int Pan_A_Speed = 0;
int Tilt_A_Speed = 0;
int Pan_Softstart = 0;
int Tilt_Softstart = 0;
int Pan_Target_Speed = 0;
int Tilt_Target_Speed = 0;
int Positioning_Sign = 0;
int Pan_Location_Speed = A_MaxSpeed;
int Tilt_Location_Speed = A_MaxSpeed * 0.6;
int Speed_Rotate = 0;
float Pan_PWM = 0;
u32 PWM_P = 0;
float Tilt_PWM = 0;
u32 PWM_T = 0;
float Pan_Kp = 0;
float Pan_Ki = 0;
float Pan_Kd = 0;
float Tilt_Kp = 0;
float Tilt_Ki = 0;
float Tilt_Kd = 0;
int Pan_CMD_Count = 1;
int Tilt_CMD_Count = 1;
int Optocoupler_Sem = 0;
int Relay_Switch = 0;
u8 Pan_Timer_Start = 1;
u8 Tilt_Timer_Start = 1;//20220615增加计时区间
u32 Velocity_Pan = 0;
u32 Velocity_Tilt = 0;
u8 ID_Dial = 0x01;
u16 Error_Record = 0;
u8 EE_Compare[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
u8 AlarmOut1 = OFF;
u8 AlarmOut2 = OFF;
u16 TimeAndData=DATE_AS_INT; //获取系统编译时间

Memory_Struct   USART1_RX_BOX;
Memory_Struct   USART2_RX_BOX;
Memory_Struct   USART3_RX_BOX;
Memory_Struct   USART1_TX_BOX;
Memory_Struct   USART2_TX_BOX;
Memory_Struct   USART3_TX_BOX;
Memory_Struct   Analyze_BOX;
Memory_Struct   Analyze_BOX2;
Memory_Struct   Alarm_BOX;
Memory_Struct   CAN_BOX;
CanTxMsg        TxMessage;
CanRxMsg        RxMessage;
//***********************************************************************************
//
//                                  Semaphores
//
//***********************************************************************************
OS_SEM  Sem_USART1_RX;
OS_SEM  Sem_USART2_RX;
OS_SEM  Sem_USART3_RX;
OS_SEM  Sem_USART_TX;
OS_SEM  Sem_USART1_TX;
OS_SEM  Sem_USART2_TX;
OS_SEM  Sem_USART3_TX;
OS_SEM  Sem_CAN;
OS_SEM  Sem_Analyze;
//串口解析传递函数
void Post_Information_Function(OS_SEM TASK_SEM,Memory_Struct *TASK_Communication_Memory,u16 command,u32 data)
{
    OS_ERR err;
    if(TASK_Communication_Memory -> Statistic_number < Memory_Finger_NUM)
    {
        TASK_Communication_Memory -> Memory_command[TASK_Communication_Memory -> memory_finger] = command;
        TASK_Communication_Memory -> Memory_Data[TASK_Communication_Memory -> memory_finger] = data;
        TASK_Communication_Memory -> Statistic_number ++;
        TASK_Communication_Memory -> memory_finger ++;
        if(TASK_Communication_Memory->memory_finger >= Memory_Finger_NUM)
        {
            TASK_Communication_Memory->memory_finger = 0;
        }
        OSSemPost(&TASK_SEM,OS_OPT_POST_1,&err);
    }
}
void Pend_Information_Function(Memory_Struct *TASK_Communication_Memory,FNC abc)
{
    if(TASK_Communication_Memory -> Statistic_number > 0)
    {
        abc();
        TASK_Communication_Memory -> Statistic_number --;
        TASK_Communication_Memory -> extract_finger ++;
        if(TASK_Communication_Memory -> extract_finger >= Memory_Finger_NUM)
        {
            TASK_Communication_Memory -> extract_finger = 0;
        }
    }
}
//串口发送传递函数
void Post_Information_Function_TX(OS_SEM TASK_SEM,Memory_Struct *TASK_Communication_Memory,u16 command,u8 *data,u8 len)
{
 //   OS_ERR err;
    u16 i = 0;
    if(TASK_Communication_Memory -> Statistic_number < Memory_Finger_NUM)
    {
        for(i = 0; i < len; i ++)
        {
            TASK_Communication_Memory -> Memory_Data_1[TASK_Communication_Memory -> memory_finger][i] = data[i];
        }
        TASK_Communication_Memory -> Memory_command[TASK_Communication_Memory -> memory_finger] = command;
        TASK_Communication_Memory -> Memory_Data[TASK_Communication_Memory -> memory_finger] = len;
        TASK_Communication_Memory -> Statistic_number ++;
        TASK_Communication_Memory -> memory_finger ++;
        if(TASK_Communication_Memory->memory_finger >= Memory_Finger_NUM)
        {
            TASK_Communication_Memory->memory_finger = 0;
        }
 //       OSSemPost(&TASK_SEM,OS_OPT_POST_1,&err);
    }
}
void Pend_Information_Function_TX(Memory_Struct *TASK_Communication_Memory)
{
    if(TASK_Communication_Memory -> Statistic_number > 0)
    {
        TASK_Communication_Memory -> Statistic_number --;
        TASK_Communication_Memory -> extract_finger ++;
        if(TASK_Communication_Memory -> extract_finger >= Memory_Finger_NUM)
        {
            TASK_Communication_Memory -> extract_finger = 0;
        }
    }
}
//******************************************************************************************************************************
void Post_Information_Function_CAN(OS_SEM TASK_SEM,Memory_Struct *TASK_Communication_Memory,u8 *data,u8 start,u8 end)
{
    u16 i = 0,j = 0;
    OS_ERR err;
    if(TASK_Communication_Memory -> Statistic_number < Memory_Finger_NUM)
    {
        for(j = 0; j < Memory_Data_NUM; j ++)
        {
            TASK_Communication_Memory -> Memory_Data_1[TASK_Communication_Memory -> memory_finger][j] = 0;
        }
        for(i = start; i < end; i ++)
        {
            TASK_Communication_Memory -> Memory_Data_1[TASK_Communication_Memory -> memory_finger][i - start] = data[i];
        }
        TASK_Communication_Memory -> Statistic_number ++;
        TASK_Communication_Memory -> memory_finger ++;
        if(TASK_Communication_Memory->memory_finger >= Memory_Finger_NUM)
        {
            TASK_Communication_Memory->memory_finger = 0;
        }
        OSSemPost(&TASK_SEM,OS_OPT_POST_1,&err);
    }
}
//**********************************************************************************
u8 USART_DMA_Send_Data(DMA_Stream_TypeDef* DMAy_Streamx,u8 *tx_buf,u8 *data,u8 length)
{
    u8 flag = 1;
    memcpy(tx_buf,data,length);
    DMA_Enable(DMAy_Streamx,length);
    return flag;
}
u8 Check_Sum(u8 *data,u8 start,u8 end)//和计算
{
    u8 sum = 0,i;
    for(i = start; i < end; i ++)
    {
        sum += data[i];
    }
    return sum;
}
int My_strlen(char *src)
{
    int len = 0;
    while(*src++ != '\0')//
    {
        len++;
    }
    return len;//返回字符串长度
}
//***********************************************************************************
//
//                                  Delay Functions
//
//***********************************************************************************
//微秒延时函数
void Delay_xUS(u16 us)
{
    u16 temp;
    temp = us * 84;
    while(temp > 0)
    {
        temp --;
    }
}
//毫秒延时函数
void Delay_xMS(u16 ms)
{
    u16 i,j;
    for(i = 0; i < ms; i ++)
    {
        for(j = 0; j < 8400; j ++)
        {
            ;
        }
    }
}
void SoftReset(void)
{
    __set_FAULTMASK(1); // 关闭所有中断
    NVIC_SystemReset(); // 复位
}
