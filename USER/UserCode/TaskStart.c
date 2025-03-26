#include "TaskStart.h"
#include "TaskMap.h"
#include "TaskUSART1.h"
#include "TaskUSART2.h"
#include "TaskUSART3.h"
#include "TaskUSART_TX.h"
#include "TaskAnalyze.h"
#include "TaskCAN.h"
#include "TaskEncoder.h"
#include "TaskTimer.h"
#include "TaskLedState.h"
#include "hardware.h"
#include "24LCxx.h"
//任务控制块
OS_TCB  Task_Start_TCB;
OS_TCB  Task_CAN_TCB;
OS_TCB  Task_USART1_RX_TCB;
OS_TCB  Task_USART2_RX_TCB;
OS_TCB  Task_USART3_RX_TCB;
OS_TCB  Task_USART_TX_TCB;
OS_TCB  Task_Analyze_TCB;
OS_TCB  Task_Encoder_TCB;
OS_TCB  Task_Timer_TCB;
OS_TCB  Task_LedState_TCB;
//static CPU_STK  Task_CAN_Stack[Task_CAN_Stack_Size];
static CPU_STK  Task_USART1_RX_Stack[Task_USART1_RX_Stack_Size];
static CPU_STK  Task_USART2_RX_Stack[Task_USART2_RX_Stack_Size];
static CPU_STK  Task_USART3_RX_Stack[Task_USART3_RX_Stack_Size];
static CPU_STK  Task_USART_TX_Stack[Task_USART_TX_Stack_Size];
static CPU_STK  Task_Analyze_Stack[Task_Analyze_Stack_Size];
static CPU_STK  Task_Encoder_Stack[Task_Encoder_Stack_Size];
static CPU_STK  Task_Timer_Stack[Task_Timer_Stack_Size];
static CPU_STK  Task_LedState_Stack[Task_LedState_Stack_Size];
u16 key = 0;
u8 i = 0;
void Task_Start(void *p)
{
    OS_ERR err;
    CPU_SR_ALLOC();
    p = p;
    CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);    //统计任务
#endif
#ifdef CPU_CFG_INT_DIS_MEAS_EN      //如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();
#endif
#if OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
    //使能时间片轮转调度功能，时间片长度为1个系统时钟节拍，既1*5=5ms
    OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);
#endif
    OS_CRITICAL_ENTER();    //进入临界区
    //*****************************
    //初始化
    //
    EEPRom_IIC_Init();
//      GET_ID_GPIO_INIT();//获取拨码板数据
//      key = Get_ID() & 0xfff;//获取拨码板数据
//    ID_Dial = key & 0xff;
//      i = (key & 0xf00) >> 8;
//      switch (i)
//      {
//          case 0:
//              USART1_BAND = 1200;
//              USART2_BAND = 1200;
//              USART3_BAND = 1200;
//          break;
//          case 1:
//              USART1_BAND = 2400;
//              USART2_BAND = 2400;
//              USART3_BAND = 2400;
//          break;
//          case 2:
//              USART1_BAND = 4800;
//              USART2_BAND = 4800;
//              USART3_BAND = 4800;
//          break;
//          case 3:
//              USART1_BAND = 9600;
//              USART2_BAND = 9600;
//              USART3_BAND = 9600;
//          break;
//          default:
    USART1_BAND = USART_BAND_Extract(USART1_BAUDRATE);
    USART2_BAND = USART_BAND_Extract(USART2_BAUDRATE);
    USART3_BAND = USART_BAND_Extract(USART3_BAUDRATE);
//          break;
//      }
    //
    //*****************************
    NVIC_Configuration();
    //*****************************
    //喂狗
    IWDG_ReloadCounter();
    //****************************呼吸灯**********************************************
    OSTaskCreate((OS_TCB     *)&Task_LedState_TCB,                //任务控制块
                 (CPU_CHAR   *)"LedState",
                 (OS_TASK_PTR ) Task_LedState,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_LedState_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_LedState_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_LedState_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_LedState_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    //****************************串口1接收**********************************************
    OSTaskCreate((OS_TCB     *)&Task_USART1_RX_TCB,                //任务控制块
                 (CPU_CHAR   *)"USART1_RX",
                 (OS_TASK_PTR ) Task_USART1_RX,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_USART1_RX_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_USART1_RX_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_USART1_RX_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_USART1_RX_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    //****************************CAN**********************************************
//    OSTaskCreate((OS_TCB     *)&Task_CAN_TCB,                //任务控制块
//                 (CPU_CHAR   *)"CAN",
//                 (OS_TASK_PTR ) Task_CAN,                //任务函数
//                 (void       *) 0,
//                 (OS_PRIO     ) Task_CAN_PRIO,           //任务优先级
//                 (CPU_STK    *)&Task_CAN_Stack[0],         //任务堆栈基地址
//                 (CPU_STK_SIZE) Task_CAN_Stack_Size / 10,       //任务堆栈深度限位
//                 (CPU_STK_SIZE) Task_CAN_Stack_Size,            //任务堆栈大小
//                 (OS_MSG_QTY  ) 5u,
//                 (OS_TICK     ) 0u,
//                 (void       *) 0,
//                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
//                 (OS_ERR     *)&err);
    //****************************串口2接收**********************************************
    OSTaskCreate((OS_TCB     *)&Task_USART2_RX_TCB,                //任务控制块
                 (CPU_CHAR   *)"USART2_RX",
                 (OS_TASK_PTR ) Task_USART2_RX,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_USART2_RX_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_USART2_RX_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_USART2_RX_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_USART2_RX_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    //****************************串口3接收**********************************************
    OSTaskCreate((OS_TCB     *)&Task_USART3_RX_TCB,                //任务控制块
                 (CPU_CHAR   *)"USART3_RX",
                 (OS_TASK_PTR ) Task_USART3_RX,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_USART3_RX_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_USART3_RX_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_USART3_RX_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_USART3_RX_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    //****************************串口发送**********************************************
    OSTaskCreate((OS_TCB     *)&Task_USART_TX_TCB,                //任务控制块
                 (CPU_CHAR   *)"USART_TX",
                 (OS_TASK_PTR ) Task_USART_TX,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_USART_TX_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_USART_TX_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_USART_TX_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_USART_TX_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    //****************************数据解析**********************************************
    OSTaskCreate((OS_TCB     *)&Task_Analyze_TCB,                //任务控制块
                 (CPU_CHAR   *)"Analyze",
                 (OS_TASK_PTR ) Task_Analyze,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_Analyze_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_Analyze_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_Analyze_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_Analyze_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    //****************************编码器**********************************************
    OSTaskCreate((OS_TCB     *)&Task_Encoder_TCB,                //任务控制块
                 (CPU_CHAR   *)"Encoder",
                 (OS_TASK_PTR ) Task_Encoder,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_Encoder_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_Encoder_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_Encoder_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_Encoder_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    //****************************定时器**********************************************
    OSTaskCreate((OS_TCB     *)&Task_Timer_TCB,                //任务控制块
                 (CPU_CHAR   *)"Timer",
                 (OS_TASK_PTR ) Task_Timer,                //任务函数
                 (void       *) 0,
                 (OS_PRIO     ) Task_Timer_PRIO,           //任务优先级
                 (CPU_STK    *)&Task_Timer_Stack[0],         //任务堆栈基地址
                 (CPU_STK_SIZE) Task_Timer_Stack_Size / 10,       //任务堆栈深度限位
                 (CPU_STK_SIZE) Task_Timer_Stack_Size,            //任务堆栈大小
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
    OS_TaskSuspend((OS_TCB*)&Task_Start_TCB,&err);      //挂起启动任务
    OS_CRITICAL_EXIT(); //退出临界区
}
uint32_t USART_BAND_Extract(uint32_t USART_BAUDRATE)
{
    u8 USART_BAND_BUF[8] = {0};
    uint32_t BAND;
    BAND = EEPRom_ReadOneByte(EEPROM_Parameter,(USART_BAUDRATE - 1) * Parameter_Bytes_NUN);
    if(EEPRom_ReadOneByte(EEPROM_Parameter,(USART_BAUDRATE - 1) * Parameter_Bytes_NUN) == 0xaa)
    {
        EEPRom_ReadDataLen(EEPROM_Parameter,USART_BAUDRATE,(u8 *)USART_BAND_BUF,Parameter_Bytes_NUN);
        BAND = ((USART_BAND_BUF[5] | 0x00000000) << 16) + ((USART_BAND_BUF[6] | 0x00000000) << 8) + USART_BAND_BUF[7];
    }
    else
    {
        BAND = USART_BAND;
    }
    return BAND;
}
//
