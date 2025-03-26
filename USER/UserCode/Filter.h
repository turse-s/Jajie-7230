#ifndef __FILTER__H
#define __FILTER__H
#include "stdint.h"
#include "math.h"
//#define Filter_On_Off 0
typedef struct
{
    uint8_t count;      //计数
    uint8_t max_count;  //上限
    double value;        //数值
} Filter_DisappearsShakes_t;
typedef struct
{
    int Disposable_Switch;          //滤波器一次性开关 启动用
    int Init_Ready;                 //滤波数组初始化完成标志
    int Main_Switch;                //滤波器综合开关
    int length;                     //滤波数组长度
    double sum;                     //滤波和
    double flag_sum;                //正弦滤波总和
    double next;                    //滤波新值
    double last;                    //滤波旧值
    double error;                   //极限滤波最大值
    double resolution;              //极限滤波最小值
    int flag;                       //正弦滤波标识符
    int flag_array[255];            //正弦滤波数组
    double array[255];              //滤波数组
    int speed;                      //速度
} Filter_Data_t;
void FilterCreate(Filter_DisappearsShakes_t *filter,uint8_t filter_ceiling,double data);         //创建滤波器
double Filter_DisappearsShakes(Filter_DisappearsShakes_t *filter,double data);   //消抖滤波
double Filter(double temp_t,Filter_Data_t *data);
double Filter_Choose(Filter_Data_t *data,double CAU,unsigned int PWM);
#endif
