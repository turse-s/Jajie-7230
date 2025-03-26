//**************************************
// File      : bsp.h
// Version   : V1.3.0
// By        : YangJing
// For       : PT_HIAC
// Device    : STM32F407VET6
//**************************************
/*******************************
云台方向：速度：上+，下-；右+，左-
          角度：上增，下减；右增，左减
          水平角度向右1~360度
          垂直角度（原点与大地平行）向上1~45度，向下360~315度
********************************/
#ifndef _parameter_H
#define _parameter_H
//***********************************************************************************
//
//                                  Includes
//
//***********************************************************************************
#include "includes.h"
#include "delay.h"
#include "TaskMap.h"
#include "stm32f4xx_iwdg.h"
#include "Filter.h"
#include "PID.h"
//***********************************************************************************
//
//                                  Defines
//
//***********************************************************************************
//#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */
//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);      /*使能SWD 禁用JTAG*/
//缂撳瓨闀垮害蹇呴』鏄?32鐨勫?嶆暟
#define PT_Version  0x2700  //程序版本号
//#define PT_LOAC_2215  //修改设备型号需要调整output文件名称，PT2315
#define PT_HIAC_7280    //修改设备型号需要调整output文件名称，PT7280
//#define PT_EXF
//#define ECO           //低速模式
#define TURBO       //超频模式
//#define LASER_SETUP   //如果使用激光灯则将改配置打开，否则将其注释掉，注意激光灯配置的IO与编码器IO共用，这在之后的版本中需要改进
//#define REVERSE_PAN //特殊电机
//#define REVERSE_TILT //特殊电机
//#define SH		//视辉
#define ZS		//兆晟
//#define ZS02    //兆晟中天
//#define ZS03
#define HY          //寰宇
//#define NewU2				//新U2 独立485模式
#define INDU2				//独立 U2
#define RV					//Return in Visca
//#define AUX_VISCA		//辅助开关走visca
//#define FOC					//电机走CAN通信 foc

#ifdef ZS
//#define NewU2				//新U2 独立485模式
#define RV					//Return in Visca
#endif
#ifdef ZS03
#define HY
#endif
//#define Output
#ifdef PT_EXF
#define PT_HIAC_7280
#endif
#ifdef PT_LOAC_2215
#define PID_PARAM1//PID参数配置1 适用于2315系列产品
#define RS_PARAM1//减速距离配置1 适用于2315系列产品
#define DIR_PARAM1//电机方向配置
#define MIN_SPEED_HIGH//最低速：高
#define LIMIT_PARAM1//限位配置1 适用于2315系列产品
//#define PT_Type     0x090b    //机器型号（16进制
#define PT_Type     0x1526  //机器型号（16进制 青春版
#endif
#ifdef PT_HIAC_7280
#define PID_PARAM2//PID参数配置2 适用于7280系列产品
#define RS_PARAM2//减速距离配置2 适用于7280系列产品
#define DIR_PARAM2//电机方向配置
#define MIN_SPEED_LOW//最低速：低
#define LIMIT_PARAM2//限位配置2 使用与7280系列产品
//#define REVERSE_TILT //特殊电机
#define PT_Type     0x1c71  //机器型号（16进制
#endif
#ifdef FOC
#define REVERSE_PAN //特殊电机
#define REVERSE_TILT //特殊电机
#endif
//#define Encoder_Position_BUF_A_LEN  64
//#define Encoder_Position_BUF_B_LEN  32
//#define track_offect        0x0800c000
//#define data_buf_len_addr   track_offect                                                                        //0x0800c000
//#define pan_data_a_addr     track_offect + 4                                                                    //0x0800c004
//#define pan_data_b_addr     track_offect + 4 + Encoder_Position_BUF_A_LEN * 4                                   //0x0800c104
//#define tilt_data_a_addr    track_offect + 4 + Encoder_Position_BUF_A_LEN * 4 + Encoder_Position_BUF_B_LEN * 4  //0x0800c184
//#define tilt_data_b_addr    track_offect + 4 + Encoder_Position_BUF_A_LEN * 8 + Encoder_Position_BUF_B_LEN * 4  //0x0800c284
//#define pan_offset_addr     track_offect + 4 + Encoder_Position_BUF_A_LEN * 8 + Encoder_Position_BUF_B_LEN * 8  //0x0800c304
//#define tilt_offset_addr    track_offect + 8 + Encoder_Position_BUF_A_LEN * 8 + Encoder_Position_BUF_B_LEN * 8  //0x0800c308
//#define offset_flag_addr    track_offect + 12 + Encoder_Position_BUF_A_LEN * 8 + Encoder_Position_BUF_B_LEN * 8 //0x0800c30c
extern u8 PT_ID;
#define USART_BAND  9600
extern u32 USART1_BAND;
extern u8 Swtich_va;
extern u32 USART2_BAND;
extern u32 USART3_BAND;
#define Encoder_Forward     0x00
#define Encoder_Inversion   0x01
extern u8 Pan_Encoder_Direction;
extern u8 Tilt_Encoder_Direction;
//通用参数
#define High_Leve   1   //通用引脚高电平
#define Low_Leve    0   //通用引脚低电平
#define Rated(a)    a   //通用常数
#define ON      Low_Leve
#define OFF     High_Leve
#define watcher_pause      1    //守望位暂停
#define watcher_continue     0  //守望位继续
//编码器读取时序延时参数
#define Encoder_Delay_US    0x01
//通用定时器参数
#define TIM10_ARR   20 - 1
#define TIM10_PSC   8400 - 1
//调速定时器参数
#define Motor_arr   500 - 1   //定时器计数值
#define Motor_psc   8400 - 1    //定时器分频值
//PWM定时器参数
#define Motor_PWM_arr   3999   //定时器计数值
#define Motor_PWM_psc   0    //定时器分频值
#define Motor_PWM_Max   700     //PWM限定最大电流值
#define Max_PWM     Motor_PWM_arr + 1
#define Min_PWM     210
//PID系数
#ifdef PID_PARAM1
#define P_Kp  0.02     //比例系数 20220606
#define P_Ki  0.02      //积分系数
#define P_Kd  0.05      //微分系数
#define T_Kp  0.10      //比例系数
#define T_Ki  0.05      //积分系数
#define T_Kd  0.08      //微分系数
#else
#ifdef PID_PARAM2
#define P_Kp  0.05      //比例系数
#define P_Ki  0.02      //积分系数
#define P_Kd  0.08      //微分系数
#define T_Kp  0.1      //比例系数
#define T_Ki  0.035      //积分系数
#define T_Kd  0.08      //微分系数
#endif
#endif
#ifdef RS_PARAM1
#define ICMU_RS_P       16
#define MT6825_RS_P     40
#define AS5600_RS_P        35
#define ICMU_RS_T       8
#define MT6825_RS_T     16
#define AS5600_RS_T        20
#else
#ifdef RS_PARAM2
#define ICMU_RS_P       35     //原80
#define MT6825_RS_P     30
#define AS5600_RS_P     25
#define ICMU_RS_T       20
#define MT6825_RS_T     20
#define AS5600_RS_T     15
#else
#define ICMU_RS_P       20
#define MT6825_RS_P     8
#define AS5600_RS_P     25
#define ICMU_RS_T       8
#define MT6825_RS_T     6
#define AS5600_RS_T     15
#endif
#endif
extern float Pan_Kp;
extern float Pan_Ki;
extern float Pan_Kd;
extern float Tilt_Kp;
extern float Tilt_Ki;
extern float Tilt_Kd;
extern int Pan_CMD_Count;
extern int Tilt_CMD_Count;
extern int Optocoupler_Sem;
extern int Relay_Switch;
#ifdef TURBO
#define Task_Encoder_Cycle_Time 10
#else
#define Task_Encoder_Cycle_Time 50
#endif
#define Task_Timer_Cycle_Time   10
//***********************************************************************************
//                                  Motors
//***********************************************************************************
#ifdef DIR_PARAM1
#define DIR_Left    High_Leve    //?
#define DIR_Right   Low_Leve   //?
#define DIR_Up    High_Leve     //?
#define DIR_Down  Low_Leve    //?
#else
#ifdef DIR_PARAM2
#define DIR_Left    Low_Leve    //?
#define DIR_Right   High_Leve   //?
#ifdef PT_EXF
#define DIR_Up    Low_Leve      //?
#define DIR_Down  High_Leve   //?
#else
#define DIR_Up    High_Leve     //?
#define DIR_Down  Low_Leve    //?
#endif
#endif
#endif
#define BRK_ON  Low_Leve
#define BRK_OFF High_Leve
#define EN_ON   High_Leve
#define EN_OFF  Low_Leve
//***********************************************************************************
//
//                                                              Time Settings
//
//
#define YEAR ((((__DATE__ [7] - '0') * 10 + (__DATE__ [8] - '0')) * 10 \
    + (__DATE__ [9] - '0')) * 10 + (__DATE__ [10] - '0'))
#define MONTH (__DATE__ [2] == 'n' ? 0 \
    : __DATE__ [2] == 'b' ? 1 \
    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 2 : 3) \
    : __DATE__ [2] == 'y' ? 4 \
    : __DATE__ [2] == 'n' ? 5 \
    : __DATE__ [2] == 'l' ? 6 \
    : __DATE__ [2] == 'g' ? 7 \
    : __DATE__ [2] == 'p' ? 8 \
    : __DATE__ [2] == 't' ? 9 \
    : __DATE__ [2] == 'v' ? 10 : 11)
#define DAY ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 \
    + (__DATE__ [5] - '0'))
#define DATE_AS_INT (((YEAR - 2000) * 13 + MONTH+1) * 32 + DAY)
extern u16 TimeAndData;
extern double FOV;
//***********************************************************************************
//
//                                  Parameters
//
//***********************************************************************************
//总线通讯
#define Unassigned_Address  0
#define Assigned_Address    !Unassigned_Address
extern u8 Assigned_Address_Flag;
#define CAN_Bcast   0x0000//广播地址
extern u16 Local_CAN_StdId;
extern u32 Local_CAN_ExtId;
extern u8 Local_CAN_IDE;//标准标识符
extern u8 Local_CAN_RTR;//数据帧
extern u8 Local_CAN_DLC;
extern u8 Local_CAN_BUF[8];
//**********************************************************************
#define Memory_Finger_NUM   20
#define Memory_Data_NUM     100
typedef struct /*串口数据传输使用*/
{
    u8 memory_finger;           //存储指针
    u8 extract_finger;          //读取指针
    u8 Statistic_number;        //内存块使用数
    u32 Memory_command[Memory_Data_NUM];
    u32 Memory_Data[Memory_Data_NUM];
    u8 Memory_Data_1[Memory_Finger_NUM][Memory_Data_NUM];
} Memory_Struct;
extern u8 Axial;//透传标识//调整护照支架
extern u8 Delete_Preset_FLag;
extern u16 Delete_Preset_Time;
extern u8 Delete_Data_FLag;
extern u16 Delete_Data_Time;
#define Protocol_PELCO_D    0x07
#define Protocol_PELCO_P    0x08
#define Protocol_VISCA      0xff
//USART1
extern u8 USART1_RX_Byte;
extern u8 USART1_RX_Byte_Num;
#define DMA_USART1_RX_BUF_SIZE  Memory_Data_NUM
#define DMA_USART1_TX_BUF_SIZE  70
extern u8 DMA_USART1_RX_Buf[DMA_USART1_RX_BUF_SIZE];
extern u8 DMA_USART1_TX_Buf[DMA_USART1_TX_BUF_SIZE];
extern u8 USART1_Use_DMA_Tx_Flag;
extern u8 USART1_RX_Buf[DMA_USART1_RX_BUF_SIZE];
extern u16 USART1_data_pointer;
extern u16 USART1_dispose_data_pointer;
extern u8 USART1_First_Frame_Flag;
extern u8 Angle_Accuracy_Flag;
//USART2
extern u8 USART2_RX_Byte;
extern u8 USART2_RX_Byte_Num;
#define DMA_USART2_RX_BUF_SIZE   Memory_Data_NUM
#define DMA_USART2_TX_BUF_SIZE   Memory_Data_NUM
extern u8 DMA_USART2_RX_Buf[DMA_USART2_RX_BUF_SIZE];
extern u8 DMA_USART2_TX_Buf[DMA_USART2_TX_BUF_SIZE];
extern u8 USART2_Use_DMA_Tx_Flag;
extern u8 USART2_RX_Buf[DMA_USART2_RX_BUF_SIZE];
extern u16 USART2_data_pointer;
extern u16 USART2_dispose_data_pointer;
//USART3
extern u8 USART3_RX_Byte;
extern u8 USART3_RX_Byte_Num;
#define DMA_USART3_RX_BUF_SIZE   Memory_Data_NUM
#define DMA_USART3_TX_BUF_SIZE   Memory_Data_NUM
extern u8 DMA_USART3_RX_Buf[DMA_USART3_RX_BUF_SIZE];
extern u8 DMA_USART3_TX_Buf[DMA_USART3_TX_BUF_SIZE];
extern u8 USART3_Use_DMA_Tx_Flag;
extern u8 USART3_RX_Buf[DMA_USART3_RX_BUF_SIZE];
extern u16 USART3_data_pointer;
extern u16 USART3_dispose_data_pointer;
extern u8 USART3_Protocol_Type;
extern u8 Empty_Buf[10];
#define VISCA_CAM_Zoom_Check_Timer  60
extern u16 VISCA_CAM_Zoom_Flag;
//USART_TX
#define USART_TX_BUF_SIZE   100
extern u8 USART_TX_BUF[USART_TX_BUF_SIZE];
//**********************************************************************
//**********************************************************************
#ifdef FOC
#define A_MaxSpeed  10000
#define A_MinSpeed  5
#define FocMaxSpeed 50
#else
#ifdef ECO
#define A_MaxSpeed  3750
#else
#ifdef TURBO
#define A_MaxSpeed  900
#else
#define A_MaxSpeed  5000
#endif
#endif

#ifdef TURBO
#define A_MinSpeed  10
#else
#ifdef MIN_SPEED_HIGH
#define A_MinSpeed  50
#endif
#ifdef MIN_SPEED_LOW
#define A_MinSpeed  20
#endif
#endif
#endif
//**********************************
#define WaitTime_Offset     200
#define Tour_WaitTime_Max   3000
#define Tour_WaitTime_Mid   1500
#define Tour_WaitTime_Min   200
#define Scan_WaitTime_Max   3000
#define Scan_WaitTime_Mid   1500
#define Scan_WaitTime_Min   200
#define Watcher_WaitTime_Max    3000
#define Watcher_WaitTime_Mid    1500
#define Watcher_WaitTime_Min    200
extern u16 Stay_Time;
#define Speed_Offset        40
#define Tour_Speed_Max      A_MaxSpeed
#define Tour_Speed_Mid      A_MaxSpeed / 2
#define Tour_Speed_Min      50
#define Scan_Speed_Max      A_MaxSpeed
#define Scan_Speed_Mid      A_MaxSpeed / 2
#define Scan_Speed_Min      50
#define Watcher_Speed_Max   A_MaxSpeed
#define Watcher_Speed_Mid   A_MaxSpeed / 2
#define Watcher_Speed_Min   50
typedef enum
{
		Tour_Scan_Disable,   
		Tour_Enable,         
		Scan_Enable,              
		Motor_Adaptation,    
		Rotating,            
		Skin_Fruit_ON,
		Skin_Fruit_Pause,
		Tracking_ON,					//花样扫描开始
		EEPROM_CHECK,
		Auto_Tracking					//智能追踪
}special_function_status;

extern u8 Special_Function;
extern u8 Watcher_Switch;
extern u8 watcher_pause_state;
extern u8 Tour_Num;
extern u16 Tour_Time;//巡检预置位停留时间
extern u16 Tour_Speed_Pan;
extern u16 Tour_Speed_Tilt;
extern u8 Scan_Num;
extern u16 Scan_Time_Left;//扫描停留时间
extern u16 Scan_Time_Right;//扫描停留时间
extern u16 Scan_Speed;
extern u16 Watcher_Time;
extern u16 Watcher_Speed;
extern short Tracking_Speed_Pan;
extern short Tracking_Speed_Tilt;
extern double Watcher_Angle_Pan;
extern double Watcher_Angle_Tilt;
extern double Section_Border1_Angle;
extern double Section_Border2_Angle;
extern int EEPROM_checknum0;
extern int EEPROM_checknum1;
extern int current_time;
extern float Scale_Velocity_Pan;
extern float Scale_Velocity_Tilt;
extern u8 Speed_Set_Satus;
//**********************************
//commond_res from usart1
#define Direction_Stop          0x0000
#define Direction_Up            0x0008
#define Direction_Down          0x0010
#define Direction_Left          0x0004
#define Direction_Right         0x0002
#define Direction_Up_Left       0x000c
#define Direction_Up_Right      0x000a
#define Direction_Down_Left     0x0014
#define Direction_Down_Right    0x0012
#define AUX_Switch_ON           0x0009
#define AUX_Switch_OFF          0x000B
#define ZOOM_Tele               0x0020
#define ZOOM_Wide               0x0040
#ifdef ZS03
#define ZOOM_Set								0x004f
#endif
#define FOCUS_Far               0x0080
#define FOCUS_Near              0x0100
//#define Laser                   0x0101
#define Laser_ONOFF             0x0101
#define Laser_Current_Change    0x0102
#define Laser_Current_Set       0x0103
#define Laser_Step_Change       0x0104
#define Laser_Position_Set      0x0105
#define Laser_Reset             0x0106
#define IRIS_ON                 0x0200
#define IRIS_OFF                0x0400
#define Preset_Set              0x0003
#define Preset_Delete           0x0005
#define Preset_Extract          0x0007
#define Set_Angle_Pan           0x004b
#define Set_Angle_Tilt          0x004d
#define Query_Angle_Pan         0x0051
#define Query_Angle_Tilt        0x0053
#define Query_ZOOM              0x0055
#define Query_ZOOM_Max          0x0155
#define Query_ZOOM_Min          0x0255
#define Query_Versions_PT       0x2808  //鏌ヨ鍥轰欢鐗堟湰
#define Query_Type_PT           0x2810  //鏌ヨ浜戝彴鍨嬪彿
#define Query_Time_Current 0x2811 //鏌ヨ褰撳墠鏃堕棿
#define Query_Encoder_PT        0x2812  //鏌ヨ缂栫爜鍣ㄥ瀷鍙?
#define Set_PID_Pan           0x3001
#define Set_PID_Tilt          0x3002
#define Setup_Filter            0x0412
#define Reset_Lens              0x2097
#define Set_3D                  0x20f1
#define Origin_Machine              0xa501
#define Limit_Machine_Up            0xa503
#define Limit_Machine_Down          0xa505
#define Tilt_Angle_Direction          0xa507
#define Clear_Parameter_Presets        0xa509
#define Clear_Flash                    0xa50b
#define Stop_U2                        0xa50d
#define Adaptation                     0xa511
#define Speed_Set                      0xa513
#define Query_Speed                    0xa515
#define Query_Speed_Pan_Max         	0xa515
#define Query_Speed_Pan_Min         	0xa517
#define Query_Speed_Tilt_Max        	0xa519
#define Query_Speed_Tilt_Min        	0xa51b
#define FOV_Config                       0xa51d
#define Up_Sidedown                     0xa51f
#define Limit_Pan_Switch							0xa521
#define EEPROM_Return                   0xa523
#ifdef ZS //兆晟
#define Extract_Number_Require  0x4096
extern u16 Extract_Number;
#endif
#ifdef RV
#define Return_Intime_Visca     0x4097
#endif
#ifdef ZS02
extern double Pan_Angle_PowerBoot;
extern double Tilt_Angle_PowerBoot;
extern u8 POST_Switch;
#define Query_Version_ZS									0x0045
#endif
extern u8 PTZ_Inverted;
#ifdef SH
#define Left_Stay_Time              0x2003
#define Right_Stay_Time             0x2005
#define Scan_Speed_Set              0x2007
#define Scan_Border_Left            0x2009
#define Scan_Border_Right           0x200b
#define Start_Scan                  0x200d
#define Delete_Scan                 0x200f
#define Rotate_Speed                0x2017
#define Rotate_Start                0x2029
#define Patrol_Start                0x20a0
#define Patrol_End                  0x20a1
#define Patrol_Setup                0x20a2
#define Patrol_Gradient                0x20a4
#define Patrol_Config               0x20a5
typedef struct
{
    u16 time_left;
    u16 time_right;
    int speed;
    double border_left;
    double border_right;
		u16 Zoom_Position_Right;
		u16 Zoom_Position_Left;
} Scan_Data;

typedef struct
{
    double P_data_1;//开始点P值
    double P_data_2;//开始点T值
    double T_data_1;//结束点P值
    double T_data_2;//结束点T值
    int speed;//速度级
    int speed_p;
    int speed_t;
    int dir;//方向 0顺时针 1逆时针
    double grads;//梯度值
    int status;//状态值 0启动 1暂停 2停止
    int situation;//当前进行步骤
    double T_data_temp;//垂直角度值中间值
		double P_data_pause;
		double T_data_pause;
        u16 Zoom_pause;
    int flag;//
		int last;
} Patrol_Data;
extern Scan_Data Sweep[5];
extern Patrol_Data Cruise;
typedef enum
{
    start,
    pause,
    go_on,
    recover
}patrol_pause_status;
    
#endif

#ifdef HY
//线扫功能?
#define Scan_Stay_Time								0x73
#define Scan_Border_Left            0x0011
#define Scan_Border_Right           0x0013
#define Start_Scan                  0x001B
#define Delete_Scan                 0x001D
//水平旋转
#define Rotate_Contrary_Start				0x0016
//模仿学习功能
#define Imitate_Record							0x001F
#define Imitate_Stop								0x0021
#define Imitate_start								0x0023
#define Imitate_size								800
#define Imitate_ON									1
#define Imitate_OFF									2
#define Route_ON										3
#define Route_OFF										3
#define Imitate_Address							0x08060000
#define Skin_Start									0x65
#define Skin_Direction_Set					0x006F
#define Skin_UP_Set									0x006B
#define Skin_Down_Set								0x0067

extern int Leading_mod;
//Machine Learing & Tarcking
typedef struct
{
    double Pan;
    double Tilt;    
}Angle_Type;
typedef struct
{
    Angle_Type Start;
    Angle_Type Stop;
    u16 Zoom; 
    u16 Direction;
    u16 Speed;
    u8 id;
    int counter;
}Learning;
extern Learning Tracking[25];
extern int StartML;
extern u16 Scan_Zoom_Left;
extern u16 Scan_Zoom_Right;
typedef struct
{
    u16 time_left;
    u16 time_right;
    u16 speed;
    double border_left;
    double border_right;
		u16 Zoom_Position_Right;
		u16 Zoom_Position_Left;
} Scan_Data;
extern Scan_Data Sweep[6];
typedef struct 
{
	double Up;							//上限
	double	Down;						//下限
	u8	Id;							//ID
	u8  Tilt_Dif;				//每次移动差值
	int	Speed;						//速度
	double Forerunner;				//先行者位置
	u8  Direction;					//方向
	u8 	Statt_Mode;					//运行状态
}Skin;
 extern Skin Fruit;
extern u8 Imitation_Process;
#endif
//Special_Function
//#define Machine_Origin              1000
#define USART1_BAUDRATE             1001
#define USART2_BAUDRATE             1002 
#define USART3_BAUDRATE             1003
#define Limit_Machine_Up_EE         1004
#define Limit_Machine_Down_EE       1005
#define Tilt_Angle_Direction_EE     1006
#define USART3_Protocol_Type_EE     1007
#define PID_Manual                  1008
#define Filter_EE                   1009
#define Max_Speed                   1010
#define	Inversion_Normal						1011
#define Limit_Pan                   1012
#define POST_Switch_EE									1013
//FLASH
#define Flash_addr                  0x800c000
#define Machine_Origin              0
#define Limit_Machine_Up_FLASH      8
#define Limit_Machine_Down_FLASH    12
//Special_Preset
#define Origin_Pan                  201
#define Origin_Tilt                 202
#define Angle_Return_Mode           203
#define Tour_Speed_Regulation       204
#define Tour_Speed_Level            205
#define Tour_StayTime_Regulation    206
#define Tour_StayTime_Level         207
#define Watcher_StayTime_Regulation 208
#define Watcher_StayTime_Level      209
#define Scan_Speed_Regulation       210
#define Scan_Speed_Level            211
#define Return_Query_485            212
#define Return_Type                 213
#define Laser_Adjust_Switch_UNV     162
#define Laser_Adjust_Switch         214
#define USART3_Protocol_Type_Switch 221
#define Self_Adaption_Switch        222
#define Home_Position               223
#define Limit_Right                 226
#define Limit_Left                  227
#define Watcher_Set                 228
#define Tour_Switch                 229
#define Delete_Data                 230
#define Section_Border1             231
#define Section_Border2             232
#define Scan_Switch                 233
#define Scan_Switch_1               234//扫大于180度扇区
#define Limit_Down_Set              235
#define Limit_Up_Set              236
#ifdef HY
#define Skin_Fruit									260
#define Imitate_Information					250
#endif
#ifdef ZS02
#define Inversion                   253//吊装
#endif
extern u8 U2_switch;
extern u8 Preset_Extract_Flag;
extern u8 Preset_Extract_num;
//
#define Return_RealTime     0x01
#define Return_Query        0x02
extern u8 Return_Mode;
#define Return_Position 0x01
#define Return_Angle    0x02
extern u8 Return_Data_Type;
#define Self_Adaption_Enable    0x01
#define Self_Adaption_Disable   0x00
extern u8 Self_Adaption;
//**********************************
//commond_res from usart3
#define ZOOM_Position_Data      0x005d
#define ZOOM_Position_Max_Data  0x0169
#define ZOOM_Position_Min_Data  0x0069
#define FOCUS_Position_Data     0x0065
extern u16 Zoom_Position;
extern u16 Zoom_Position_Max;
extern u16 Zoom_Position_Min;
extern u16 Focus_Position;
extern u16 Versions_Lens;
extern u16 Scan_Zoom_Left;
extern u16 Scan_Zoom_Right;
extern u16 AUX_Position;
extern int ZS_speed;
//Preset
extern u16 Preset_Num[1024];
extern u8 Preset_Number;
extern u8 Preset_Empty;
#define Preset_Bytes_NUN    16
#define Parameter_Bytes_NUN 8
#define EEPROM_Parameter    0   //特殊预置位存储在0号EEPROM中
#define EEPROM_Preset       1   //用户预置位存储在1号EEPROM中
//Encoder
#define Encoder_Normal  0x00
#define Encoder_Error   0x01
#define Encoder_Drop    0x02
extern u8 Pan_Encoder_State;
extern u8 Tilt_Encoder_State;
#define Circle_Value    360.000 //整圈角度值
//#ifdef PT_LOAC_2215
//#define Encoder_Range   262144  //编码器量程
//#define Angle_Accuracy  0.005  //角度精度
//#else
//#define Encoder_Range   262144  //编码器量程
//#define Angle_Accuracy  Circle_Value / Encoder_Range  //角度精度
//#endif
#define Section_Range   4096    //编码器分区量程
extern double Angle_Decimal;
#define Tour_Scan_Accumulation  256
extern u32 encoder_counter;
extern double Pan_Encoder_Range;
extern double Pan_Angle_Accuracy;
extern int track_offset;
extern double Pan_Origin_User;
extern double Pan_Real_Angle;
extern double Pan_Real_Angle_1;
extern double Pan_Real_Angle_Last;
extern double Pan_Current_Angle_User;
extern double Pan_Target_Angle;
extern double Pan_Current_Angle_Machine;
extern double Pan_Machine_Origin_Angle;
extern double Tilt_Encoder_Range;
extern double Tilt_Angle_Accuracy;
extern double Tilt_Origin_User;
extern double Tilt_Real_Angle;
extern double Tilt_Real_Angle_1;
extern double Tilt_Real_Angle_Last;
extern double Tilt_Current_Angle_User;
extern double Tilt_Target_Angle;
extern double Tilt_Target_Angle_Last;
extern double Tilt_Current_Angle_Machine;
extern double Tilt_Machine_Origin_Angle;
extern Filter_DisappearsShakes_t Pan_Angle_Filter;
extern Filter_DisappearsShakes_t Tilt_Angle_Filter;
extern int Rotating_Direction;
#define Encoder_AS5311  ((uint8_t)(1))
#define Encoder_AS5600  ((uint8_t)(2))
#define Encoder_ICMU    ((uint8_t)(3))
#define Encoder_MT6825  ((uint8_t)(4))
extern u16 pan_encoder_type;
extern u16 tilt_encoder_type;
extern double pan_angle_query;
extern double tilt_angle_query;
extern u8 Pan_Init_Flag;
extern u8 Pan_Section_Counter;
extern u8 Pan_Section_Counter_Check;
extern u32 Pan_Current_Data;//0x80000
extern u16 Pan_Encoder_Data;//
extern u32 Pan_Encoder_Data_Last;//上电编码器值
extern u32 Pan_Machine_Origin;
extern u32 Pan_Encoder_Position_Save;
extern u32 Pan_Encoder_Position_Current;
extern u32 Pan_Encoder_Position_Real;
extern u32 Tilt_Encoder_Position_Real;
extern int pan_track_offset;
extern int tilt_track_offset;
extern int pan_track_err;
extern int tilt_track_err;
extern u8 Tilt_Init_Flag;
extern u8 Tilt_Section_Counter;
extern u8 Tilt_Section_Counter_Check;
extern u32 Tilt_Current_Data;//0x80000
extern u32 Tilt_Current_Data_Last;
extern u32 Tilt_Encoder_Position_Save;
extern u16 Tilt_Encoder_Data;//
extern u32 Tilt_Encoder_Data_Last;//上电编码器值
extern u32 Tilt_Machine_Origin;
extern u32 Tilt_Encoder_Position_Current;
#define Pan_User_Limit_Enable   0xB1
#define Pan_User_Limit_Disable  0xA0
extern u8 Pan_User_Limit_Switch;
extern double Pan_User_Limit_L;
extern double Pan_User_Limit_R;
extern u8 Pan_Soft_Limit_FLAG;
//#define Tilt_Angle_45           4500
//#define Tilt_Angle_50           5000
//#define Tilt_Angle_60           6000
//#define Tilt_User_Limit         Tilt_Angle_45
//#define Tilt_User_Limit_Up      0x200  //45
//#define Tilt_User_Limit_Down    0xe00  //315
#ifdef LIMIT_PARAM1
#define Tilt_Angle_Limit_Up      45.0   //20220222zhangxuezhi，由原来的45.0改为60.0
#define Tilt_Angle_Limit_Down    270.0  //20220222zhangxuezhi，由原来的290.0改为275.0
#define Tilt_Angle_Limit_Up_default      45.0   //20220222zhangxuezhi，增加默认限位角度，如果不设定限位角度则按照该角度进行配置
#define Tilt_Angle_Limit_Down_default    270.0  //20220222zhangxuezhi，增加默认限位角度
#else
#ifdef LIMIT_PARAM2
#define Tilt_Angle_Limit_Up      60.0   //
#define Tilt_Angle_Limit_Down    300.0  //
#define Tilt_Angle_Limit_Up_default      60.0   //20220222zhangxuezhi，增加默认限位角度，如果不设定限位角度则按照该角度进行配置
#define Tilt_Angle_Limit_Down_default    305.0  //20220222zhangxuezhi，增加默认限位角度
#else
#define Tilt_Angle_Limit_Up      60.0   //
#define Tilt_Angle_Limit_Down    300.0  //
#define Tilt_Angle_Limit_Up_default      60.0   //20220222zhangxuezhi，增加默认限位角度，如果不设定限位角度则按照该角度进行配置
#define Tilt_Angle_Limit_Down_default    300.0  //20220222zhangxuezhi，增加默认限位角度
#endif
#endif
extern double Tilt_Limit_Up;
extern double Tilt_Limit_Down; 
typedef enum
{
    Tilt_Limit_None,         
    Tilt_Limit_Over_Up,      
    Tilt_Limit_Over_Down,    
    Tilt_Limit_Over,         
    Tilt_Limit_Near_Up,      
    Tilt_Limit_Near_Down    
}Tilt_Limit_Status;
typedef enum
{
    Pan_Limit_None,            
    Pan_Limit_Over,         
    Pan_Limit_Near_L,      
    Pan_Limit_Near_R    
}Pan_Limit_Status;

#define Pan_Limit_Break         5
#define Tilt_Limit_Brake        10
#define Tilt_Limit_Drop         0x10
extern u8 Tilt_Hard_Limit_FLAG;
extern u8 Tilt_Soft_Limit_FLAG;
extern u8 Origin_Machine_Flag;
extern u8 Origin_Machine_Record[8];
#define Tilt_State_Normal_Up    0x4
#define Tilt_State_Normal_Down  0x1
#define Tilt_State_Normal_All   0x0
#define Tilt_State_Abnormal_Up    0x6
#define Tilt_State_Abnormal_Down  0x3
//*******************************************************
#define Pan_MAX_Sampling_Period_Position    436
#define Tilt_MAX_Sampling_Period_Position   218
//缂栫爜鍣ㄥ崟鍖哄搴?
#define Encoder_Sector_Position 8192 / 2
#define pi 3.14159265f
//typedef struct Encoder_Position_BUF
//{
//  u32 Position_Current;
//  u32 Position_Last;
//  u32 Position_Sampling_Period;
//  u16 BUF_1_Cursor;
//  u16 BUF_2_Cursor;
//    u32 Encoder_Position_BUF_A[Encoder_Position_BUF_1_LEN];
//  u32 Encoder_Position_BUF_B[Encoder_Position_BUF_2_LEN];
//}Encoder_Position_BUF_t;
//extern Encoder_Position_BUF_t Encoder_Position_BUF_Pan;
//extern Encoder_Position_BUF_t Encoder_Position_BUF_Tilt;
//*******************************************************
//Motor
#define Rocker_Motion       0xf000
#define Location_Motion     0x0fff
#define AutoTracking_Motion	0x0ff0
#define Location_Motion_Scan    0x01ff
#define Pan_Angle_Report        0x0001
#define Tilt_Angle_Report       0x0002
extern u16 Pan_Motion_Mode;
extern u16 Tilt_Motion_Mode;
#define OutPlace    0x0
#define InPlace     0x1
extern u8 Place_InOut_Pan;
extern u8 Place_InOut_Tilt;
extern double Pan_SetAngle_RetardSection;
extern double Tilt_SetAngle_RetardSection;
//#ifdef PT_LOAC_2215
//#define Pan_SetAngle_RetardSection  80//角度设定减速区间
//#define Tilt_SetAngle_RetardSection 30//角度设定减速区间
//#else
//#define Pan_SetAngle_RetardSection   16//角度设定减速区间
//#define Tilt_SetAngle_RetardSection   8//角度设定减速区间
//#endif
extern float ratio_add;
extern float ratio_subtract_pan;
extern float ratio_subtract_tilt;
//double ratio_idle_pan;
//double ratio_idle_tilt;
extern int Pan_A_Speed;
extern int Tilt_A_Speed;
extern int Pan_Softstart;
extern int Tilt_Softstart;
extern int Pan_Target_Speed;
extern int Tilt_Target_Speed;
extern int Pan_Location_Speed;
extern int Tilt_Location_Speed;
extern int Pan_A_MaxSpeed;
extern int Tilt_A_MaxSpeed;
extern int Pan_A_MinSpeed;
extern int Tilt_A_MinSpeed;
extern int Pan_Ultimate_MaxSpeed;
extern int Tilt_Ultimate_MaxSpeed;
extern int Pan_Ultimate_MinSpeed;
extern int Tilt_Ultimate_MinSpeed;
extern int Speed_Rotate;
#ifdef FOC
extern int Speedmode;
extern u8 can_SpeedMod_Cmd[8];
#define rocker 0x00
#define location	0x01
#endif
extern float Speed_Scale_Pan;
extern float Speed_Scale_Tilt;
extern int Positioning_Sign;
#define Motor_State_Stop        0x00
#define Motor_State_Operation   0x01
extern u8 Pan_Timer_Start;
extern u8 Tilt_Timer_Start;
extern u8 Filter_Switch_Pan;
#define Max_PWM   Motor_PWM_arr + 1
extern float Pan_PWM;
extern u32 PWM_P;
extern float Tilt_PWM;
extern u32 PWM_T;
extern u32 Velocity_Pan;
extern u32 Velocity_Tilt;
extern u8 ID_Dial;
extern u8 AlarmOut1;
extern u8 AlarmOut2;
extern u16 Error_Record;
extern u8 EE_Compare[8];
extern Memory_Struct    USART1_RX_BOX;
extern Memory_Struct    USART2_RX_BOX;
extern Memory_Struct    USART3_RX_BOX;
extern Memory_Struct    USART1_TX_BOX;
extern Memory_Struct    USART2_TX_BOX;
extern Memory_Struct    USART3_TX_BOX;
extern Memory_Struct    Analyze_BOX;
extern Memory_Struct    Analyze_BOX2;
extern Memory_Struct    Alarm_BOX;
extern Memory_Struct    CAN_BOX;
extern CanTxMsg         TxMessage;
extern CanRxMsg         RxMessage;
//***********************************************************************************
//
//                                  Semaphores
//
//***********************************************************************************
extern OS_SEM   Sem_USART1_RX;
extern OS_SEM   Sem_USART2_RX;
extern OS_SEM   Sem_USART3_RX;
extern OS_SEM   Sem_USART_TX;
extern OS_SEM   Sem_USART1_TX;
extern OS_SEM   Sem_USART2_TX;
extern OS_SEM   Sem_USART3_TX;
extern OS_SEM   Sem_CAN;
extern OS_SEM   Sem_Analyze;
//EncoderPortInitialize
void Encoder_SSI_Init(u8 port);
//void CRC32_Init(void);
typedef void (*FNC)(void);
void Post_Information_Function(OS_SEM TASK_SEM,Memory_Struct *TASK_Communication_Memory,u16 command,u32 data);
void Post_Information_Function_TX(OS_SEM TASK_SEM,Memory_Struct *TASK_Communication_Memory,u16 command,u8 *data,u8 len);
void Pend_Information_Function_TX(Memory_Struct *TASK_Communication_Memory);
void Post_Information_Function_CAN(OS_SEM TASK_SEM,Memory_Struct *TASK_Communication_Memory,u8 *data,u8 start,u8 end);
void Pend_Information_Function(Memory_Struct *TASK_Communication_Memory,FNC abc);
u8 USART_DMA_Send_Data(DMA_Stream_TypeDef* DMAy_Streamx,u8 *tx_buf,u8 *data,u8 length);
u8 Check_Sum(u8 *data,u8 start,u8 end);//和计算
int My_strlen(char *src);
//Delays
void Delay_xUS(u16 us);
void Delay_xMS(u16 ms);
void SoftReset(void);
u16 Get_ID(void);
double HotPursuit(double goal,double now,double step,int dir);
int Up_Or_Down(double angle1, double angle2);
uint32_t USART1_BaudRate(u16 data);
u8 USART1_ADDR(u16 data);
#endif
