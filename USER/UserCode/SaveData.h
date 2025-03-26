#ifndef _SAVEDATA__H
#define _SAVEDATA__H
#include "includes.h"
u8 SaveData_C(uint8_t flash_page,u8 *para,u8 len);    //保存数据
u8 ReadData(uint8_t flash_page,u8 *para,u8 len);   //读取数据
#endif
