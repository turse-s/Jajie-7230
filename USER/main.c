#include "includes.h"
#include "parameters.h"
#include "delay.h"
#include "TaskStart.h"
//#include "Configuration.h"
//#include "Configuration_adv.h"

static CPU_STK  Task_Start_Stack[Task_Start_Stack_Size];
#ifdef Output
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif
int main(void)
{
    
    /*－－－－－－－－－－－－－－－－－－　系统初始化代码　－－－－－－－－－－－－－－－－－－*/
	OS_ERR err;
	CPU_SR_ALLOC();
    delay_init(168);
    OSInit(&err);		//初始化UCOSIII
	OS_CRITICAL_ENTER();//进入临界区
    //创建开始任务
	OSTaskCreate((OS_TCB 	* )&Task_Start_TCB,		//任务控制块
                 (CPU_CHAR	* )"Start", 		//任务名字
                 (OS_TASK_PTR )Task_Start, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )Task_Start_PRIO,     //任务优先级
                 (CPU_STK   * )&Task_Start_Stack[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)Task_Start_Stack_Size/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)Task_Start_Stack_Size,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目，为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
    OS_CRITICAL_EXIT();	//推迟临界区
	OSStart(&err);  //开启UCOSIII
	while(1);
}

