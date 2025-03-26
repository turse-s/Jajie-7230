#include "TaskUSART_TX.h"
#include "hardware.h"
#include "TaskUSART3.h"
#include "TaskUSART2.h"
#include "TaskUSART1.h"
//Command Set
u8 CAM_Zoom_Stop[6] = {0x81,0x01,0x04,0x07,0x00,0xFF};
u8 CAM_Zoom_Tele[6] = {0x81,0x01,0x04,0x07,0x02,0xFF};
u8 CAM_Zoom_Wide[6] = {0x81,0x01,0x04,0x07,0x03,0xFF};
u8 CAM_Zoom_Tele_Speed[6] = {0x81,0x01,0x04,0x07,0x20,0xFF};
u8 CAM_Zoom_Wide_Speed[6] = {0x81,0x01,0x04,0x07,0x30,0xFF};
u8 CAM_Zoom_Direct[9] = {0x81,0x01,0x04,0x47,0x00,0x00,0x00,0x00,0xFF};
u8 CAM_Focus_Far[6] = {0x81,0x01,0x04,0x08,0x02,0xFF};
u8 CAM_Focus_Near[6] = {0x81,0x01,0x04,0x08,0x03,0xFF};
u8 CAM_Focus_Tele_Speed[6] = {0x81,0x01,0x04,0x08,0x20,0xFF};
u8 CAM_Focus_Wide_Speed[6] = {0x81,0x01,0x04,0x08,0x30,0xFF};
u8 CAM_Focus_Direct[9] = {0x81,0x01,0x04,0x48,0x00,0x00,0x00,0x00,0xFF};
u8 CAM_Auto_Focus[6] = {0x81,0x01,0x04,0x38,0x02,0xFF};
u8 CAM_ICR_ON[6] = {0x81,0x01,0x04,0x01,0x02,0xFF};
u8 CAM_ICR_OFF[6] = {0x81,0x01,0x04,0x01,0x03,0xFF};
u8 CAM_IRIS_RESET[6]    = {0x81,0x01,0x04,0x0B,0x00,0xFF};
u8 CAM_IRIS_UP[6]       = {0x81,0x01,0x04,0x0B,0x02,0xFF};
u8 CAM_IRIS_DOWN[6]     = {0x81,0x01,0x04,0x0B,0x03,0xFF};
u8 CAM_IRIS_Direct[9]   = {0x81,0x01,0x04,0x4B,0x00,0x00,0x00,0x00,0xFF};
//Inquiry Command
u8 CAM_ZoomPosInq[5] = {0x81,0x09,0x04,0x47,0xFF};
u8 CAM_FocusPosInq[5] = {0x81,0x09,0x04,0x48,0xFF};
float CAM_ZoomPosMax = 16384;
float CAM_ZoomPosMin = 0;
u16 CAM_ZoomPosCur = 0;
u8 Zoom_Position_Record[8] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
u8 Zoom_Position_Extract[8] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#ifdef ZS
u8 Position_Arrival[5] = {0x81,0x01,0x08,0x00,0xFF};
#endif
void Task_USART_TX(void *p)
{
    OS_ERR err;
//    OSSemCreate ((OS_SEM* )&Sem_USART_TX,
//                 (CPU_CHAR*   )"USART_TX",
//                 (OS_SEM_CTR)0,
//                 (OS_ERR* )&err);
//    OSSemCreate ((OS_SEM* )&Sem_USART1_TX,
//                 (CPU_CHAR*   )"USART1_TX",
//                 (OS_SEM_CTR)0,
//                 (OS_ERR* )&err);
//    OSSemCreate ((OS_SEM* )&Sem_USART2_TX,
//                 (CPU_CHAR*   )"USART2_TX",
//                 (OS_SEM_CTR)0,
//                 (OS_ERR* )&err);
//    OSSemCreate ((OS_SEM* )&Sem_USART3_TX,
//                 (CPU_CHAR*   )"USART3_TX",
//                 (OS_SEM_CTR)0,
//                 (OS_ERR* )&err);
    p = p;
    while(1)
    {
//        OSSemPend(&Sem_USART_TX,0,OS_OPT_PEND_BLOCKING,0,&err);
        USART_TX_Function();
//        Pend_Information_Function_TX(&USART_TX_BOX,USART_TX_Function);
        OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
    }
}
void USART_TX_Function(void)
{
    if(USART1_TX_BOX.Statistic_number > 0 && !USART1_Use_DMA_Tx_Flag)
    {
#ifdef FOC
        if(Return_Mode == Return_Query_485)
					USART1_EN = 1;
				USART1_Use_DMA_Tx_Flag = USART_DMA_Send_Data(USART1_DMA_Steam_TX,DMA_USART1_TX_Buf,USART1_TX_BOX.Memory_Data_1[USART1_TX_BOX.extract_finger],USART1_TX_BOX.Memory_Data[USART1_TX_BOX.extract_finger]);
					USART1_EN = 0;
#else
				USART1_Use_DMA_Tx_Flag = USART_DMA_Send_Data(USART1_DMA_Steam_TX,DMA_USART1_TX_Buf,USART1_TX_BOX.Memory_Data_1[USART1_TX_BOX.extract_finger],USART1_TX_BOX.Memory_Data[USART1_TX_BOX.extract_finger]);				
#endif
		}
    if(USART2_TX_BOX.Statistic_number > 0 && !USART2_Use_DMA_Tx_Flag)
    {
        USART2_Use_DMA_Tx_Flag = USART_DMA_Send_Data(USART2_DMA_Steam_TX,DMA_USART2_TX_Buf,USART2_TX_BOX.Memory_Data_1[USART2_TX_BOX.extract_finger],USART2_TX_BOX.Memory_Data[USART2_TX_BOX.extract_finger]);
    }
    if(USART3_TX_BOX.Statistic_number > 0 && !USART3_Use_DMA_Tx_Flag)
    {
        USART3_Use_DMA_Tx_Flag = USART_DMA_Send_Data(USART3_DMA_Steam_TX,DMA_USART3_TX_Buf,USART3_TX_BOX.Memory_Data_1[USART3_TX_BOX.extract_finger],USART3_TX_BOX.Memory_Data[USART3_TX_BOX.extract_finger]);
    }
}
