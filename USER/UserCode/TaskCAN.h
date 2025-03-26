#ifndef _TaskCAN_H
#define _TaskCAN_H
#include "parameters.h"
#define normal  0xffff0000
#define upgrade 0x00110000
#define Upgrade_CAN_ID  0x08010000
//#define Succeed 0x0000
//#define Failure 0x0001
//#define Assign_Address  0x0003
//#define Query_Angle     0x0004
//#define Query_Version   0x0005
//void Task_CAN(void *p);
//void Task_CAN_PostBack(void *p);
//void CAN1_RX_Data(u8 *data_buf,u8 len);
//void CAN_Function(void);
//void CAN_PostBack_Function(void);
//void Valid_Data(char *title,char *detail,u8 title_num,u8 detail_num);
//void Data_PostBack(u16 PostBack_Type);
//u8 PostBack_Founction(char *data,u8 len);
//void PostBack_USART(u8 *data);
u8 CAN1_Send_Msg(u32 CAN_StdId,u32 CAN_ExtId,u8 CAN_IDE,u8 CAN_RTR,u8 CAN_DLC,u8 *msg,u8 start);
void iap_load_app(u32 appxaddr);
void APP_GOTO_IAP(u32 upgrade_flag_addr,u32 *upgrade_flag_data,u32 upgrade_flag_data_len);
//void SoftReset(void);
#endif
