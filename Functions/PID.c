#include "TaskMotor.h"
#include "TaskEncoder.h"
#include "hardware.h"
#include "TaskUSART_TX.h"
#include "math.h"
#include "PID.h"

ppid_data Pan_ppid, Tilt_ppid;
float RealSpeedPID_P(int ts,int cs,ppid_data *ppid)//ts:targetspeed cs:currentspeed
{
		if(ts*cs<0)//收到异向速度
		{	
				ppid->prev_error =0;
				ppid->integral = 0;
		}
		float err=ts-cs;
		ppid->integral += err;
		float derivative = err - ppid->prev_error;
		float output = ppid->Kp * err + ppid->Ki * ppid->integral + ppid->Kd * derivative;
		ppid->prev_error = err;												
		return output;
}

float RealSpeedPID_I(int ts,int cs,ipid_data *ipid)
{
		if(ts*cs<0)//收到异向速度
		{	
				ipid->prev_error =0;
				ipid->prev_prev_error = 0;
				ipid->prev_output = 0;
		}	
    float error = ts-cs;  // 计算当前误差
    float delta_error = error - ipid->prev_error;  // 计算误差的变化量
    float double_delta_error = error - 2 * ipid->prev_error + ipid->prev_prev_error;  // 计算误差的二阶变化量
    float delta_output = ipid->Kp * delta_error + ipid->Ki * error + ipid->Kd * double_delta_error;  // 计算控制量的增量
    float output = ipid->prev_output + delta_output;  // 计算当前输出
    ipid->prev_prev_error = ipid->prev_error;  // 更新上上次的误差
    ipid->prev_error = error;  // 更新上一次的误差
    ipid->prev_output = output;  // 更新上一次的输出
    return output;		
}

void PID_PWM(int ts,int cs,ppid_data *ppid,int addr)
{
		if (addr)//垂直
		{
			if (ts==0)
			{
				if(cs==0)
				{
					ppid->prev_error =0;
					ppid->integral = 0;
					Motor_Tilt_BRK(BRK_ON);	
				}	
			}
			else
			    {
					if(ts > 0)//
					Motor_Tilt_DIR(DIR_Down);
					else if(ts < 0)//
					Motor_Tilt_DIR(DIR_Up);
					Motor_Tilt_BRK(BRK_OFF);
                }
		}
		else//水平
		{
			if (ts==0)
			{
				if(cs==0)
				{
					ppid->prev_error =0;
					ppid->integral = 0;
					Motor_Pan_BRK(BRK_ON);
				
				}
				else
				{
					if(ts > 0)//
					Motor_Pan_DIR(DIR_Right);
					else if(ts < 0)//
					Motor_Pan_DIR(DIR_Left);
					Motor_Pan_BRK(BRK_OFF);
				}
			}
		}
		int pwm;
		pwm = (int)RealSpeedPID_P(ts,cs,ppid);
		if(abs(pwm)>Max_PWM)
			pwm = Max_PWM;
		if(addr)
		TIM_SetCompare2(TIM9,abs(pwm));
		else
		TIM_SetCompare1(TIM9,abs(pwm));
}

//生成一个给ppidpan和ppidtild赋值的函数
void PID_Init(ppid_data *ppid,float Kp,float Ki,float Kd)
{
		ppid->Kp = Kp;
		ppid->Ki = Ki;
		ppid->Kd = Kd;	
}	
