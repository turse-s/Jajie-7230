#ifndef _TaskEncoder_H
#define _TaskEncoder_H
#include "parameters.h"
#include "JQueue.h"
#define IC_MU_CMD_ACTIVATE              0xb0000000
#define IC_MU_CMD_SDAD_Transmission     0xa6000000  //直接返回传感器position
#define IC_MU_CMD_SDAD_Status           0xf5000000
#define IC_MU_CMD_Read_REGISTER         0x97000000
#define IC_MU_CMD_Write_REGISTER        0xd2000000
#define IC_MU_CMD_REGISTER_Status_Data  0xad000000
typedef struct
{
    u8 pan_data_a_len;
    u8 pan_data_b_len;
    u8 tilt_data_a_len;
    u8 tilt_data_b_len;
    u32 offset_flag;
    int pan_offset_data;
    int tilt_offset_data;
    u32 pan_data_a[Encoder_Position_BUF_A_LEN];
    u32 pan_data_b[Encoder_Position_BUF_B_LEN];
    u32 tilt_data_a[Encoder_Position_BUF_A_LEN];
    u32 tilt_data_b[Encoder_Position_BUF_B_LEN];
} Encoder_Position_Store_t;
extern Filter_Data_t pan_data;
extern Filter_Data_t tilt_data;
extern Encoder_Position_Store_t Encoder_Position_Store;
extern JQueue_t Encoder_Position_Pan_A;
extern JQueue_t Encoder_Position_Tilt_A;
extern JQueue_t Encoder_Position_Pan_B;
extern JQueue_t Encoder_Position_Tilt_B;
extern u32 Encoder_Position_Pan_BUF_A[Encoder_Position_BUF_A_LEN];
extern u32 Encoder_Position_Pan_BUF_B[Encoder_Position_BUF_B_LEN];
extern u32 Encoder_Position_Tilt_BUF_A[Encoder_Position_BUF_A_LEN];
extern u32 Encoder_Position_Tilt_BUF_B[Encoder_Position_BUF_B_LEN];
void Task_Encoder(void *p);
u8 Pan_Encoder_Type_Check(void);
u32 Pan_Encoder_Position(u8 type);
u8 Tilt_Encoder_Type_Check(void);
u32 Tilt_Encoder_Position(u8 type);
u8 Tilt_Soft_Limit(double Angle_User,double Limit_Up,double Limit_Down);
double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,double InitialPrediction);
//void STMFLASH_Write_1(uint32_t WriteAddr,Encoder_Position_Store_t *data_t);
void Encoder_Init(void);
void Encoder_Position_Storage(void);
#endif
