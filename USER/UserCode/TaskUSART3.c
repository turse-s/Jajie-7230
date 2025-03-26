#include "TaskUSART3.h"
#include "TaskUSART2.h"
#include "TaskUSART1.h"
#include "24LCxx.h"
#include "hardware.h"
void Task_USART3_RX(void *p)
{
    OS_ERR err;
    p = p;
    USART3_INT_Init(USART3_BAND);
    OSSemCreate ((OS_SEM*   )&Sem_USART3_RX,
                 (CPU_CHAR* )"Sem_USART3_RX",
                 (OS_SEM_CTR)0,
                 (OS_ERR*   )&err);
//    USART3_Protocol_Type = Protocol_VISCA;
    while(1)
    {
        OSSemPend(&Sem_USART3_RX,0,OS_OPT_PEND_BLOCKING,0,&err);
//        Pend_Information_Function(&USART3_RX_BOX,USART3_RX_Function);
        USART3_RX_Function();
    }
}
//*********************************************
//接收命令解析
u16 USART3_byte_num = 0;
u8 USART3_byte_buf[100];
u8 USART3_Type = 0;
extern u16 VISCA_FLAG;
void USART3_RX_Function(void)
{
    u8 PT_Position_Record[16] = {0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    for(; USART3_data_pointer != USART3_dispose_data_pointer;)
    {
        USART3_dispose_data_pointer = USART3_dispose_data_pointer % DMA_USART3_RX_BUF_SIZE;
        if(USART3_byte_num == 0)
        {
            if(USART3_RX_Buf[USART3_dispose_data_pointer] == 0xff)
            {
                USART3_byte_buf[USART3_byte_num] = USART3_RX_Buf[USART3_dispose_data_pointer];
                USART3_byte_num = 1;
                USART3_Type = 0;
            }
            else if(USART3_RX_Buf[USART3_dispose_data_pointer] == 0x90)
            {
                USART3_byte_buf[USART3_byte_num] = USART3_RX_Buf[USART3_dispose_data_pointer];
                USART3_byte_num = 1;
                USART3_Type = 1;
            }
            else
            {
                USART3_byte_num = 0;
            }
        }
        else
        {
            if(USART3_Type == 1)
            {
                USART3_byte_buf[USART3_byte_num ++] = USART3_RX_Buf[USART3_dispose_data_pointer];
//                USART3_byte_num ++;
                if(USART3_RX_Buf[USART3_dispose_data_pointer] == 0xff)
                {
                    if(USART3_byte_buf[1] == 0x50 && USART3_byte_buf[6] == 0xff)
                    {
                        Zoom_Position = 0;
                        Zoom_Position |= USART3_byte_buf[2];
                        Zoom_Position <<= 4;
                        Zoom_Position |= USART3_byte_buf[3];
                        Zoom_Position <<= 4;
                        Zoom_Position |= USART3_byte_buf[4];
                        Zoom_Position <<= 4;
                        Zoom_Position |= USART3_byte_buf[5];
                        Zoom_Position_Max = 0x4000;
                        Zoom_Position_Min = 0;
//                        if (USART3_Protocol_Type == Protocol_VISCA)
//                        {
//                            USART3_Protocol_Type = Protocol_VISCA;
//                        }
//                        else
//                        {
//                            USART3_Protocol_Type = Protocol_VISCA;
                            PT_Position_Record[1] = 0;
                            PT_Position_Record[2] = 0;
                            PT_Position_Record[3] = 0;
                            PT_Position_Record[4] = 0;
                            PT_Position_Record[5] = 0;
                            PT_Position_Record[6] = 0;
                            PT_Position_Record[7] = USART3_Protocol_Type;
                            EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record,Parameter_Bytes_NUN);
//                        }
//                        Self_Adaption = Self_Adaption_Enable;
                    }
                    if(USART3_byte_buf[1] == 0x38 && USART3_byte_buf[2] == 0xff)
                    {
                        Zoom_Position_Max = 0x4000;
                        Zoom_Position_Min = 0;
                        Zoom_Position = 0;
//                        if (USART3_Protocol_Type == Protocol_VISCA)
//                        {
//                            USART3_Protocol_Type = Protocol_VISCA;
//                        }
//                        else
//                        {
//                            USART3_Protocol_Type = Protocol_VISCA;
                            PT_Position_Record[1] = 0;
                            PT_Position_Record[2] = 0;
                            PT_Position_Record[3] = 0;
                            PT_Position_Record[4] = 0;
                            PT_Position_Record[5] = 0;
                            PT_Position_Record[6] = 0;
                            PT_Position_Record[7] = USART3_Protocol_Type;
                            EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record,Parameter_Bytes_NUN);
//                        }
//                        Self_Adaption = Self_Adaption_Enable;
                    }
										if(USART3_byte_buf[1] == 0x02 && USART3_byte_buf[5] == 0xff)
										{
												AUX_Position = USART3_byte_buf[2];
										}
                    USART3_byte_num = 0;
                }
								if(USART3_byte_buf[1] == 0x07 && USART3_byte_buf[2] == 0x06 && USART3_byte_buf[3] == 0x03)//智能追踪
								{
										Tracking_Speed_Pan = ((USART3_byte_buf[4] &0x0f)<<12) + ((USART3_byte_buf[5] &0x0f)<<8) + ((USART3_byte_buf[6] &0x0f)<<4) + ((USART3_byte_buf[7] &0x0f));
										Tracking_Speed_Pan = ~Tracking_Speed_Pan+1;
										//Tracking_Speed_Pan /= 100;
										Tracking_Speed_Tilt = ((USART3_byte_buf[8] &0x0f)<<12) + ((USART3_byte_buf[9] &0x0f)<<8) + ((USART3_byte_buf[10] &0x0f)<<4) + ((USART3_byte_buf[11] &0x0f));
										Tracking_Speed_Tilt = ~Tracking_Speed_Tilt+1;
										//Tracking_Speed_Tilt /= 100;	
										Special_Function = Auto_Tracking;
								}
            }
            else
            {
                USART3_byte_buf[USART3_byte_num] = USART3_RX_Buf[USART3_dispose_data_pointer];
                USART3_byte_num ++;
                if(USART3_byte_num > 6)
                {
                    if(USART3_byte_buf[6] == ((USART3_byte_buf[1] + USART3_byte_buf[2] + USART3_byte_buf[3] + USART3_byte_buf[4] + USART3_byte_buf[5]) & 0xff))
                    {
//                        if (USART3_Protocol_Type == Protocol_PELCO_D)
//                        {
//                            USART3_Protocol_Type = Protocol_PELCO_D;
//                        }
//                        else
//                        {
//                            USART3_Protocol_Type = Protocol_PELCO_D;
                            PT_Position_Record[1] = 0;
                            PT_Position_Record[2] = 0;
                            PT_Position_Record[3] = 0;
                            PT_Position_Record[4] = 0;
                            PT_Position_Record[5] = 0;
                            PT_Position_Record[6] = 0;
                            PT_Position_Record[7] = USART3_Protocol_Type;
                            EEPRom_WriteDataLen(EEPROM_Parameter,USART3_Protocol_Type_Switch,PT_Position_Record,Parameter_Bytes_NUN);
//                        }
                        switch(((u16)USART3_byte_buf[2] << 8) + USART3_byte_buf[3])
                        {
                        case ZOOM_Position_Data:
                        {
                            Zoom_Position = ((u16)USART3_byte_buf[4] << 8) + USART3_byte_buf[5];
                        }
                        break;
                        case ZOOM_Position_Max_Data:
                        {
                            Zoom_Position_Max = ((u16)USART3_byte_buf[4] << 8) + USART3_byte_buf[5];
                        }
                        break;
                        case ZOOM_Position_Min_Data:
                        {
                            Zoom_Position_Min = ((u16)USART3_byte_buf[4] << 8) + USART3_byte_buf[5];
                        }
                        break;
                        case FOCUS_Position_Data:
                        {
                            Focus_Position = ((u16)USART3_byte_buf[4] << 8) + USART3_byte_buf[5];
                        }
                        break;
                        case 0x2091:
                        {
                            Versions_Lens = ((u16)USART3_byte_buf[4] << 8) + USART3_byte_buf[5];
                        }
                        break;
                        default:
                        {
                            //            USART_SendData(USART1,0x55);
                            //            while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET){}
                        } break;
                        }
                    }
                    USART3_byte_num = 0;
                }
            }
        }
        USART3_dispose_data_pointer ++;
        USART3_dispose_data_pointer = USART3_dispose_data_pointer % DMA_USART3_RX_BUF_SIZE;
    }
}
