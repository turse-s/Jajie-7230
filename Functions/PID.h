#ifndef __PID__H
#define __PID__H

typedef struct
{
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float setpoint;  // 设定值
    float integral;  // 积分项
    float prev_error;  // 上一次的误差
		int pwm;
}ppid_data;//位置式pid

typedef struct {
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数
    float setpoint;  // 设定值
    float prev_error;  // 上一次的误差
    float prev_prev_error;  // 上上次的误差
    float prev_output;  // 上一次的输出
		int pwm;
}ipid_data;//增量式pid

extern int AutoPWM_P,AutoPWM_T;
extern ppid_data Pan_ppid, Tilt_ppid;

void PID_PWM_Pan(int ts,int cs,ppid_data *ppid);
void PID_PWM_Tilt(int ts,int cs,ppid_data *ppid);
void PID_Init(ppid_data *ppid,float Kp,float Ki,float Kd);

#endif
