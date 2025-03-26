#include "TaskMotor.h"
#include "TaskEncoder.h"
#include "hardware.h"
#include "TaskUSART_TX.h"
#include "math.h"
////*******************************************************
////位置式PID算法                                     //***
//    P_A_Integral += P_A;                            //***
//    Pan_PWM =    Pan_Kp * P_A                       //***
//               + Pan_Ki * P_A_Integral              //***
//               + Pan_Kd * (P_A - P_A_next);             //***
//    P_A_next = P_A;                                     //***
////*******************************************************
////*******************************************************
////增量式PID算法                                     //***
//    Pan_PWM +=    Pan_Kp * (P_A - P_A_next)             //***
//                + Pan_Ki * P_A                      //***
//                + Pan_Kd * (P_A - 2* P_A_next + PLL_A); //***
//    PLL_A = P_A_next;                                   //***
//    P_A_next = P_A;                                     //***
////*******************************************************
long int P_A = 0,P_A_next = 0,P_A_last = 0,P_A_Integral = 0;
long int T_A = 0,T_A_next = 0,T_A_last = 0,T_A_Integral = 0;
long int P_I = 0,T_I = 0;
float P_P,T_T;
void Motor_Speed_Pan(u8 Encoder_State,
                     u16 Motion_Mode,
                     int Target_Speed,
                     int A_Speed,
                     int Location_Speed,
                     u8 *Place_InOut,
                     double Target_Angle,
                     double Real_Angle,
                     double Real_Angle_Last,
                     u8 Soft_Limit_FLAG)
{
//    if(Encoder_State != Encoder_Drop)
//    {
    if((Motion_Mode & 0xfff) == Location_Motion || (Motion_Mode & 0xfff) == Location_Motion_Scan)
    {
        Pan_CMD_Count--;
        if (Pan_CMD_Count < -10)
        {
            Pan_CMD_Count = -10;
        }
        P_P = Motor_Pan_PWM_PID(Motor_Pan_Coordinate_Motion(Motion_Mode,Target_Angle,Real_Angle,Location_Speed,Place_InOut),A_Speed,Real_Angle,Real_Angle_Last);
    }
    if(Motion_Mode == Rocker_Motion)
    {
        P_P = Motor_Pan_PWM_PID(Target_Speed,A_Speed,Real_Angle,Real_Angle_Last);
        Pan_Timer_Start = 0;
    }
//    }
//    else
//    {
//        P_P = Motor_Pan_PWM_PID(0,A_Speed,Real_Angle,Real_Angle_Last);
//    }
}
void Motor_Speed_Tilt(u8 Encoder_State,
                      u16 Motion_Mode,
                      int Target_Speed,
                      int A_Speed,
                      int Location_Speed,
                      u8 *Place_InOut,
                      double Target_Angle,
                      double Real_Angle,
                      double Real_Angle_Last,
                      u8 Soft_Limit_FLAG)
{
    //***********************************************************************************************************************
    if((Motion_Mode & 0xfff) == Location_Motion || (Motion_Mode & 0xfff) == Location_Motion_Scan)
    {
        if(Encoder_State == Encoder_Normal)
        {
            Tilt_CMD_Count--;
            if (Tilt_CMD_Count < -10)
            {
                Tilt_CMD_Count = -10;
            }
            if(Target_Angle > Tilt_Limit_Up + 3 && Target_Angle < Tilt_Limit_Down - 3)
            {
                T_T = Motor_Tilt_PWM_PID(0,A_Speed,Real_Angle,Real_Angle_Last);
            }
            else if (Target_Angle > Tilt_Limit_Up && Target_Angle < Tilt_Limit_Down)
            {
                if (Target_Angle > Tilt_Limit_Up && Target_Angle < 180)//??????????,??????
                {
                    Motor_Tilt_PWM_PID(Motor_Tilt_Coordinate_Motion(Motion_Mode,Tilt_Limit_Up,Real_Angle,Location_Speed,Place_InOut),A_Speed,Real_Angle,Real_Angle_Last);
                }
                else if (Target_Angle < Tilt_Limit_Down && Target_Angle > 180)
                {
                    Motor_Tilt_PWM_PID(Motor_Tilt_Coordinate_Motion(Motion_Mode,Tilt_Limit_Down,Real_Angle,Location_Speed,Place_InOut),A_Speed,Real_Angle,Real_Angle_Last);
                }
            }
            else
            {
                Motor_Tilt_PWM_PID(Motor_Tilt_Coordinate_Motion(Motion_Mode,Target_Angle,Real_Angle,Location_Speed,Place_InOut),A_Speed,Real_Angle,Real_Angle_Last);
            }
        }
        else
        {
            Motor_Tilt_PWM_PID(0,A_Speed,Real_Angle,Real_Angle_Last);
        }
    }
    if(Motion_Mode == Rocker_Motion)
    {
        Tilt_Timer_Start = 0;
        if(Encoder_State != Encoder_Normal)
        {
            Target_Speed /= 3;
            //Real_Angle = 0;
        }
        if((Target_Speed > 0 && Soft_Limit_FLAG == Tilt_Limit_Over_Up) ||
                (Target_Speed < 0 && Soft_Limit_FLAG == Tilt_Limit_Over_Down))
        {
            T_T = Motor_Tilt_PWM_PID(0,A_Speed,Real_Angle,Real_Angle_Last);
        }
        else if(Target_Speed > 0 && Soft_Limit_FLAG == Tilt_Limit_Near_Up )
        {
            Target_Speed *= (fabs(Real_Angle - Tilt_Limit_Up) / Tilt_Limit_Brake);
            T_T = Motor_Tilt_PWM_PID(Target_Speed,A_Speed,Real_Angle,Real_Angle_Last);
        }
        else if(Target_Speed < 0 && Soft_Limit_FLAG == Tilt_Limit_Near_Down )
        {
            Target_Speed *= (fabs(Real_Angle - Tilt_Limit_Down) / Tilt_Limit_Brake);
            T_T = Motor_Tilt_PWM_PID(Target_Speed,A_Speed,Real_Angle,Real_Angle_Last);
        }
        else
        {
            T_T = Motor_Tilt_PWM_PID(Target_Speed,A_Speed,Real_Angle,Real_Angle_Last);
        }
    }
}
//***************************************************************************************************************
//
//          位置环
//
//***************************************************************************************************************
int Test_Pan_Speed;
double P_test,I_test;
double TSP_last;
int Motor_Pan_Coordinate_Motion(u16 Motion_Mode,double TargetAngle,double CurrentAngle,int Target_speed,u8 *Place_InOut)
{
    double dif_err = 0;
    int out_speed = 0;
    if(Pan_CMD_Count)//当前速度为0时才能进这个判断1次，如果想再进入需速度为0且目标速度为0（停止）
    {
        if (TSP_last != TargetAngle && Special_Function != Scan_Enable)
        {
            return 0;//行进途中更改目标，先降速到0再重新goto
        }
    }
    else
    {
        TSP_last = TargetAngle;
    }
    dif_err = TargetAngle - CurrentAngle;
    if (fabs(dif_err) >= 180)
    {
        dif_err -= (fabs(dif_err) / dif_err) * 360;
    }
    if(fabs(dif_err) >= Pan_Angle_Accuracy)     //误差角度小于Pan_SetAngle_RetardSection大于Pan_Angle_Accuracy时速度与误差角度成比例关系，误差越小，速度越低
    {
        if (fabs(dif_err) >= Pan_Angle_Accuracy * 5)
        {
            Pan_Timer_Start = 0;
        }
        else
        {
            Pan_Timer_Start = 1;
        }
        out_speed = 150 * fabs(dif_err);
    }
    else
    {
        out_speed = 0;
        P_A_Integral = 0;
        P_A = 0;
        P_A_next = 0;
        PWM_P = 0;
        Pan_PWM = 0;
        Motor_Pan_BRK(BRK_ON);      //当达到目标时速度降为0，并且踩刹车
        *Place_InOut = InPlace;
        Pan_Timer_Start = 1;
        Pan_Target_Speed = 0;
        Pan_Motion_Mode = Rocker_Motion;
    }
    if (out_speed > Pan_Location_Speed)
    {
        out_speed = Pan_Location_Speed;
    }
//    else if(out_speed <Pan_A_MinSpeed)
//    {
//        out_speed = Pan_A_MinSpeed;//速度限幅
//    }
    out_speed = (dif_err >= 0) ?  out_speed : ~out_speed + 1;//判断方向
    out_speed = (Motion_Mode == Location_Motion_Scan) ?  ~out_speed + 1 : out_speed;//判断是否为大于180扫描
    Test_Pan_Speed =   out_speed ;
    return      out_speed;
}
int Test_Tilt_Speed;
double TST_Last;
int Motor_Tilt_Coordinate_Motion(u16 Motion_Mode,double TargetAngle,double CurrentAngle,int Target_speed,u8 *Place_InOut)
{
    double dif_err = 0;
    int out_speed = 0;
    if(Tilt_CMD_Count)
    {
        if (TST_Last != TargetAngle && Special_Function != Scan_Enable)
        {
            return 0;//连续调用时至少调用两次才
        }
    }
    else
    {
        TST_Last = TargetAngle;
    }
    dif_err = TargetAngle - CurrentAngle;
    if (fabs(dif_err) >= 180)
    {
        dif_err -= (fabs(dif_err) / dif_err) * 360;
    }
    if(fabs(dif_err) >= Tilt_Angle_Accuracy)
    {
        if (fabs(dif_err) >= Tilt_Angle_Accuracy * 5)
        {
            Tilt_Timer_Start = 0;
        }
        else
        {
            Tilt_Timer_Start = 1;
        }
        out_speed = 150 * fabs(dif_err);//pow(dif_err,2) * k + b;
    }
    else
    {
        out_speed = 0;
        T_A_Integral = 0;
        T_A = 0;
        T_A_next = 0;
        PWM_T = 0;
        Tilt_PWM = 0;
        Motor_Tilt_BRK(BRK_ON);     //当达到目标时速度降为0，并且踩刹车
        *Place_InOut = InPlace;
        Tilt_Timer_Start = 1;
        Tilt_Target_Speed = 0;
        Tilt_Motion_Mode = Rocker_Motion;
    }
    if (out_speed > Tilt_Location_Speed)
    {
        out_speed = Tilt_Location_Speed;
    }
//    else if(out_speed < Tilt_A_MinSpeed)
//    {
//        out_speed = Tilt_A_MaxSpeed*Speed_Scale;
//    }
    out_speed = (dif_err >= 0) ?  out_speed : ~out_speed + 1;//判断方向
    out_speed = (Motion_Mode == Location_Motion_Scan) ? out_speed = ~out_speed + 1 : out_speed;//判断是否为大于180扫描
    if ((Tilt_Soft_Limit_FLAG == Tilt_Limit_Over_Up && out_speed > 0) || (Tilt_Soft_Limit_FLAG == Tilt_Limit_Over_Down && out_speed < 0))
    {
        out_speed = 0;//如果是超限位状态且目标位置是反方向则不动
    }
    Test_Tilt_Speed =   out_speed ;
    return out_speed;
}
//***************************************************************************************************************
//
//          调速函数
//
//***************************************************************************************************************
long int P_Pan,I_Pan;
int Pan_TargetSpeed_last = 0;
//#define_Increment PID_Control
float Motor_Pan_PWM_PID(int TargetSpeed,int CurrentSpeed,double Real_Angle,double Real_Angle_Last)
{
    if ((TargetSpeed * CurrentSpeed < 0))
    {
        P_A = 0;
        P_I = 0;
        P_A_next = 0;
        P_A_last = 0;
//            PWM_P = 0;
//            Pan_PWM = 0;
        TargetSpeed = 0;
    }//异向启动清零pid
    else
    {
        if (Pan_Motion_Mode == Location_Motion && TargetSpeed != Pan_Ultimate_MaxSpeed * Speed_Scale_Pan)
        {
            TargetSpeed = TargetSpeed;
            Pan_TargetSpeed_last = TargetSpeed;
        }
        else
        {
            TargetSpeed = 0.7 * Pan_TargetSpeed_last + (1 - 0.7) * TargetSpeed;
            Pan_TargetSpeed_last = TargetSpeed;
        }
    }
    P_A = (TargetSpeed - CurrentSpeed) * 5;
#ifdef Increment_PID_Control
    P_Pan = Pan_Kp * (P_A - P_A_next);
    I_Pan = Pan_Ki * P_A;
#else
    P_I += P_A;
    P_Pan = Pan_Kp * P_A;
    I_Pan = Pan_Ki * P_I;
#endif
    //*************************************************
    if(TargetSpeed != 0)//not gioing to stop
    {
#ifdef Increment_PID_Control
        Pan_PWM += (P_Pan + I_Pan);//+ Pan_Kd * (P_A - 2* P_A_next + P_A_last));
#else
        Pan_PWM = P_Pan + I_Pan;// + Pan_Kd * (P_A - P_AL);
#endif
        Pan_PWM += fabs(Pan_PWM)/Pan_PWM * 100;
        if (fabs(Pan_PWM) > Max_PWM)
        {
            Pan_PWM = fabs(Pan_PWM) / Pan_PWM * Max_PWM;
        }
        if(TargetSpeed > 0)//
        {
            Motor_Pan_BRK(BRK_OFF);
            Motor_Pan_DIR(DIR_Right);
        }
        else if(TargetSpeed < 0)//
        {
            Motor_Pan_BRK(BRK_OFF);
            Motor_Pan_DIR(DIR_Left);
        }
    }
    else//going to stop
    {
        if (CurrentSpeed == 0)
        {
            P_A = 0;
            P_I = 0;
            P_A_next = 0;
            P_A_last = 0;
            PWM_P = 0;
            Pan_PWM = 0;
            Pan_Softstart = 0;
            Motor_Pan_BRK(BRK_ON);
            //Pan_Motion_Mode = Rocker_Motion;
            Pan_CMD_Count = 1;
        }
        else if(Pan_PWM / CurrentSpeed > 0)
        {
            Pan_PWM = P_Pan + I_Pan;
//              if (abs(CurrentSpeed) >= 1200)
//              {
//                  Pan_PWM += Pan_Kp * P_A * ratio_subtract_pan / 2;
//              }
//              else
//              {
//                  Pan_PWM += Pan_Kp * P_A * ratio_subtract_pan;
//              }
        }
        else if (Pan_PWM == 0)
        {
            Motor_Pan_BRK(BRK_ON);//防止溜车
        }
        else
        {
            Pan_PWM = 0;
        }
    }
#ifdef Increment_PID_Control
    P_A_last = P_A_next;
    P_A_next = P_A;
    P_AL  = P_A;
#endif
    PWM_P = abs((int)Pan_PWM);
    /*********************************************************************************/
    TIM_SetCompare1(TIM9,PWM_P);
    return PWM_P;
}
float P_Tilt,I_Tilt;
int Tilt_TargetSpeed_last = 0;
float Motor_Tilt_PWM_PID(int TargetSpeed,int CurrentSpeed,double Real_Angle,double Real_Angle_Last)
{
    if (TargetSpeed * CurrentSpeed < 0)
    {
        T_A = 0;
        T_I = 0 ;
        T_A_next = 0;
        T_A_last = 0;
        PWM_T = 0;
        Tilt_PWM = 0;
    }//异向启动清零pid
    else
    {
        if (Tilt_Motion_Mode == Location_Motion && TargetSpeed != Tilt_A_MaxSpeed)
        {
            TargetSpeed = TargetSpeed;
            Tilt_TargetSpeed_last = TargetSpeed;
        }
        else
        {
            TargetSpeed = 0.7 * Tilt_TargetSpeed_last + (1 - 0.7) * TargetSpeed;
            Tilt_TargetSpeed_last = TargetSpeed;
        }
    }
    T_A = (TargetSpeed - CurrentSpeed) * 5;
#ifdef Increment_PID_Control
    P_Tilt = Tilt_Kp * (T_A - T_A_next);
    I_Tilt = Tilt_Ki * T_I;
#else
    T_I += T_A;
    P_Tilt = Tilt_Kp * T_A;
    I_Tilt = Tilt_Ki * T_I;
#endif
    //*************************************************
    if(TargetSpeed)//not gioing to stop
    {
#ifdef Increment_PID_Control
        Tilt_PWM += (P_Tilt + I_Tilt);
        Tilt_PWM = P_Tilt + I_Tilt;
#else
        Tilt_PWM = P_Tilt + I_Tilt;
#endif
        Tilt_PWM += fabs(Tilt_PWM) / Tilt_PWM * 100;
        //Tilt_PWM /= Tilt_Softstart;
        if (fabs(Tilt_PWM) > Max_PWM)
        {
            Tilt_PWM = fabs(Tilt_PWM) / Tilt_PWM * Max_PWM;
        }
        if(TargetSpeed > 0)//
        {
            Motor_Tilt_BRK(BRK_OFF);
            Motor_Tilt_DIR(DIR_Down);
        }
        else if(TargetSpeed < 0)//
        {
            Motor_Tilt_BRK(BRK_OFF);
            Motor_Tilt_DIR(DIR_Up);
        }
    }
    else//going to stop
    {
        if (CurrentSpeed == 0)
        {
            T_A = 0;
            T_I = 0;
            T_A_next = 0;
            T_A_last = 0;
            PWM_T = 0;
            Tilt_PWM = 0;
            Tilt_Softstart = 0;
            Motor_Tilt_BRK(BRK_ON);
            //Tilt_Motion_Mode = Rocker_Motion;
            Tilt_CMD_Count = 1;
        }
        else if(Tilt_PWM / CurrentSpeed > 0)
        {
            Tilt_PWM = P_Tilt + I_Tilt;
        }
        else if(Tilt_PWM == 0)
        {
            Motor_Tilt_BRK(BRK_ON);
        }
        else
        {
            Tilt_PWM = 0;
        }
    }
#ifdef Increment_PID_Control
    T_A_last = T_A_next;
    T_A_next = T_A;
#endif
    PWM_T = abs((int)Tilt_PWM);
    /*********************************************************************************/
    TIM_SetCompare2(TIM9,PWM_T);
    return PWM_T;
}
