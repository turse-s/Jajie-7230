#ifndef _TaskUSART_TX_H
#define _TaskUSART_TX_H
#include "parameters.h"
//Command Set
extern u8 CAM_Zoom_Stop[6];
extern u8 CAM_Zoom_Tele[6];
extern u8 CAM_Zoom_Wide[6];
extern u8 CAM_Zoom_Tele_Speed[6];
extern u8 CAM_Zoom_Wide_Speed[6];
extern u8 CAM_Zoom_Direct[9];
extern u8 CAM_Focus_Far[6];
extern u8 CAM_Focus_Near[6];
extern u8 CAM_Focus_Tele_Speed[6];
extern u8 CAM_Focus_Wide_Speed[6];
extern u8 CAM_Focus_Direct[9];
extern u8 CAM_Auto_Focus[6];
extern u8 CAM_ICR_ON[6];
extern u8 CAM_ICR_OFF[6];
extern u8 CAM_IRIS_RESET[6];
extern u8 CAM_IRIS_UP[6];
extern u8 CAM_IRIS_DOWN[6];
extern u8 CAM_IRIS_Direct[9];
//Inquiry Command
extern u8 CAM_ZoomPosInq[5];
extern u8 CAM_FocusPosInq[5];
extern float CAM_ZoomPosMax;
extern float CAM_ZoomPosMin;
extern u16 CAM_ZoomPosCur;
void Task_USART_TX(void *p);
void USART_TX_Function(void);
#endif
