#include "Filter.h"
#include "hardware.h"
#include "parameters.h"
/*****************************
* �����˲�
*****************************/
double Filter_DisappearsShakes(Filter_DisappearsShakes_t *filter,double data)
{
    if(data!=filter->value)
    {
        filter->count++;
        if(filter->count>filter->max_count)
        {
            filter->count=0;
            filter->value=data;
        }
    }
    else
    {
        filter->count=0;
    }
    return filter->value;
}
/****************************
*�����˲���
****************************/
void FilterCreate(Filter_DisappearsShakes_t *filter,uint8_t filter_ceiling,double data)
{
    filter->count=0;
    filter->max_count=filter_ceiling;
    filter->value=data;
}
/*�����˲�*/
//double Filter(double temp_t,Filter_Data_t *data)    //---------------滤波------------------20220616
//{
//        if(temp_t != 0xeeee)
//     {
//            data->next = temp_t;
//          if ((((data->next >= data->last) ? (data->next - data->last) : (data->last - data->next)) > data->resolution))
//          {
//              data->last = temp_t;
//          }
//          else
//          {
//              temp_t = data->last;
//              data->err_num ++;
//          }
//          for (int i = data->length - 1;i > 0;i--)
//                {
//                    data->array[i] = data->array[i - 1];
//                }
//                data->array[0] = temp_t;
//      }
//        else
//        {
//            return temp_t;//数据异常的时候不进行计算
//        }
//
//        for (int i = 0;i < data->length;i++)
//        {
//            data->sum += data->array [i];
//        }
//
//        temp_t = data->sum / data->length;
//        data->sum = 0;
//          return temp_t;
//}
double Filter_Choose(Filter_Data_t *data,double CAU,u32 PWM)
{
    double query;
    int time;
    time = Task_Encoder_Cycle_Time;
    //data->Init_Ready = data->length;
    if(data->Init_Ready)
    {
        data->Disposable_Switch = 1;
    }
    if(PWM !=0)
    {
        if (data->Main_Switch || data->Disposable_Switch)// 滤波开�?
        {
            query = Filter(CAU,data);//启用滤波函数，输出滤波后的数�?
        }
        else//
        {
            query = CAU;
        }
        data->Disposable_Switch = 0;//一次性开�?开启一次后关闭
    }
    else
    {
        query = data->last;//当电机不再运动且没有激活数据刷新的开关信号量输入则输出一个虚假的数据
    }
    data->speed = ((int)((data->last - query) * 1000 / time * 100) > 30000) ? data->speed : (int)((data->last - query) * 1000 / time * 100);
//    data->last = CAU;//未启用滤波函数，直接输出函数，更新data.last为最新的数据
    return query;
}
double Filter(double temp_t,Filter_Data_t *data)    //---------------滤波------------------20220616
{
    if(temp_t != 0xeeee)//检查数据合理�?
    {
        if (temp_t > 180)//判断是否大于180度，因为sin函数大于180是负数，这里进行记录，后面还原的时候再分开处理
        {
            data->flag = 1;
        }
        else
        {
            data->flag = 0;
        }
        if (data->Init_Ready)//判断滤波数组是否初始化完成，未完成直接返回真实值不开启滤�?
        {
            data->array[data->Init_Ready - 1] = sin((temp_t *pi /180 /2));
            data->flag_array[data->Init_Ready - 1] = data->flag;
            data->Init_Ready --;
            data->last = temp_t;
            return temp_t;
        }
        else
        {
            data->next = temp_t;
            if (((data->next >= data->last) ? (data->next - data->last) : (data->last - data->next)) > data->resolution)//极限值滤波，滤分辨率
            {
                data->last = temp_t;//数据变化幅度大于分辨�?采用该数�?
            }
            else
            {
                temp_t = data->last;//数据变化幅度小于分辨�?采用上一次的数据
            }
            for (int i = data->length - 1; i > 0; i--) //数组中数据下移一�?
            {
                data->array[i] = data->array[i - 1];
                data->flag_array[i] = data->flag_array[i - 1];
            }
            data->array[0] = sin((temp_t *pi /180 /2));//存入新的数据
            data->flag_array[0] = data->flag;
        }
    }
    else
    {
        return temp_t;//数据异常的时候不进行计算
    }
    for (int i = 0; i < data->length; i++) //计算总和来求平均值，同时算一�?80度左右边的权占比
    {
        data->sum += data->array [i];
        data->flag_sum += data ->flag_array[i];
    }
    temp_t = data->sum / data->length;
    data->sum = 0;
    temp_t = asin(temp_t) /pi *180;//通过反正弦函数还原角度�?
    if (data->flag_sum/ data->length >= 0.5)//如果原数据大�?80�?
    {
        temp_t = (180 -temp_t) *2;
    }
    else
    {
        temp_t = temp_t *2;
    }
    data->flag_sum = 0;
    return temp_t;
}
