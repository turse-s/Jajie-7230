#ifndef _TaskMotor_H
#define _TaskMotor_H
#include "parameters.h"
#include "PID.h"
void Motor_Speed_Pan(u8 Encoder_State,
                     u16 Motion_Mode,
                     int Target_Speed,
                     int A_Speed,
                     int Location_Speed,
                     u8 *Location_Place_InOut,
                     double Target_Angle,
                     double Real_Angle,
                     double Real_Angle_Last,
                     u8 Soft_Limit_FLAG);
void Motor_Speed_Tilt(u8 Encoder_State,
                      u16 Motion_Mode,
                      int Target_Speed,
                      int A_Speed,
                      int Location_Speed,
                      u8 *Location_Place_InOut,
                      double Target_Angle,
                      double Real_Angle,
                      double Real_Angle_Last,
                      u8 Soft_Limit_FLAG);
//µ÷ËÙº¯Êý
float Motor_Pan_PWM_PID(int TargetSpeed,int CurrentSpeed,double Real_Angle,double Real_Angle_Last);
float Motor_Tilt_PWM_PID(int TargetSpeed,int CurrentSpeed,double Real_Angle,double Real_Angle_Last);
int Motor_Pan_Coordinate_Motion(u16 Motion_Mode,double TargetAngle,double CurrentAngle,int Target_speed,u8 *Place_InOut);
int Motor_Tilt_Coordinate_Motion(u16 Motion_Mode,double TargetAngle,double CurrentAngle,int Target_speed,u8 *Place_InOut);
int Motor_Pan_Scan_Motion(double PanTargetAngle,double PanCurrentAngle,int Target_speed);
void AutoTrackingMotor(int ts,int cs,ppid_data *ppid,int addr);
extern long int I_Pan;
extern float I_Tilt;
											
#ifdef FOC
typedef union {
	unsigned char fByte[4];
	float ffloat;
}ftoByte;
void MotorSendSpeed(u8 addr,ftoByte candata);
#endif
#endif
