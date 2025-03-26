#include "laser.h"
#include "hardware.h"
#include "parameters.h"
#include "TaskUSART_TX.h"
uint8_t laser_onoff_cmd[7]          = {0xff,0x01,0x01,0x01,0x00,0x00,0x04};//开关，第四位，开0x01,�?x00
uint8_t laser_current_cmd[7]        = {0xff,0x01,0x01,0x02,0x00,0x00,0x04};//电流
uint8_t laser_current_set[7]        = {0xff,0x01,0x01,0x03,0x00,0x00,0x00};//设置电流，第四位0x00~0xff
uint8_t laser_step_decrease[7]      = {0xff,0x01,0x01,0x04,0x00,0x00,0x00};//光斑变小走指定步数，第五�?
uint8_t laser_step_increase[7]      = {0xff,0x01,0x01,0x04,0x01,0x00,0x00};//光斑变大走指定步数，第五�?
uint8_t laser_position_set[7]       = {0xff,0x01,0x01,0x05,0x00,0x00,0x00};//设置光斑位置�?x0000~0x1db0
uint8_t laser_reset[7]              = {0xff,0x01,0x01,0x06,0x00,0x00,0x08};//光斑到复位（最大）
laser_s laser_t =
{
    .laser_status = laser_off,
    .laser_position = 0,
};
double lens_angle = 0;
double laser_angle = 0;
#ifdef LASER_SETUP
void laser_handle(void)
{
    OS_ERR err;
    if(LDR_Status)//检测到黑夜状�?
    {
        if(laser_t.laser_status == laser_off)//关闭状�?
        {
            if(USART3_Protocol_Type == Protocol_VISCA)
            {
                laser_t.laser_status = laser_on;
                Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,CAM_ICR_ON,6);//将相机切换至黑夜
                OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
                Post_Information_Function(Sem_Analyze,&Analyze_BOX,Laser_ONOFF,0x0100);//打开激光器
                Post_Information_Function(Sem_Analyze,&Analyze_BOX,Laser_Current_Set,0x6000);//调整电流
            }
        }
        else//打开状�?
        {
            //摄像机zoom变化函数
            Post_Information_Function(Sem_Analyze,&Analyze_BOX,Query_ZOOM,0);
            //double Zoom_Position_t = Zoom_Position;
            lens_angle = 67.5 - 66.65 * (double)Zoom_Position / 16384;
//          //摄像机zoom分段
//          if(Zoom_Position_t >= 11047)
//          {
//              lens_angle = 134951.66 / 5337 - 7.67 * Zoom_Position_t /5337;
//          }
//          else if(Zoom_Position_t >= 6268)
//          {
//              lens_angle = 199407.45 / 4779 - 13.98 * Zoom_Position_t / 4779;
//          }
//          else
//          {
//              lens_angle = 63.39 - 40 * Zoom_Position_t / 6268;
//          }
            laser_angle = lens_angle * 1.76 + 2;
            //激光器角度分段
            if(laser_angle <= 4.78)
            {
                laser_t.laser_position = (2826 * laser_angle - 803) / 4.48;
            }
            else
            {
                laser_t.laser_position = (6400 * laser_angle + 69291.22) / 35.22;
            }
            if(laser_t.laser_position < 0)
            {
                laser_t.laser_position = 0;
            }
            if(laser_t.laser_position > 9236)
            {
                laser_t.laser_position = 9236;
            }
            Post_Information_Function(Sem_Analyze,&Analyze_BOX,Laser_Position_Set,laser_t.laser_position);
        }
    }
    else
    {
        if(laser_t.laser_status == laser_on)
        {
            laser_t.laser_status = laser_off;
            Post_Information_Function(Sem_Analyze,&Analyze_BOX,Laser_ONOFF,0x0000);//关闭激光器
            OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
            Post_Information_Function_TX(Sem_USART_TX,&USART3_TX_BOX,3,CAM_ICR_OFF,6);//将相机切换至白天
        }
    }
}
#endif
//uint16_t camera_zoom_smash_lights_position(uint16_t temp)
//{
//  uint16_t return_temp;float Proportion;
//  if(temp > 5461)
//  {
//      if(temp > 10922)
//      {
//          Proportion = (float)(temp - 2*5461) / 5462.0;
//          return_temp = PTZ_user_paramter.CAM_Zoom_Smash_Lights_two_thirds - ((u16)Proportion*(PTZ_user_paramter.CAM_Zoom_Smash_Lights_two_thirds - PTZ_user_paramter.CAM_Zoom_Smash_Lights_max));
//      }
//      else/*0~1555*/
//      {
//          Proportion = (float)(temp-5461) / 5462.0;
//          return_temp = PTZ_user_paramter.CAM_Zoom_Smash_Lights_one_thirds - ((u16)Proportion*(PTZ_user_paramter.CAM_Zoom_Smash_Lights_one_thirds - PTZ_user_paramter.CAM_Zoom_Smash_Lights_two_thirds));
//      }
//  }
//  else/*0~1555*/
//  {
//      if(temp == 0)
//      {
//          temp = 1;
//      }
//      Proportion = (float)temp / 5462.0;
//      return_temp = PTZ_user_paramter.CAM_Zoom_Smash_Lights_min - ((u16)Proportion*(PTZ_user_paramter.CAM_Zoom_Smash_Lights_min - PTZ_user_paramter.CAM_Zoom_Smash_Lights_one_thirds));
//  }
//  return return_temp;
//}
