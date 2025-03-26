#include "SaveData.h"
#include "stdbool.h"
#include "CRC_16.h"
#include "malloc.h"
extern Mallco_dev  malloc_dev;
/********************************************
 * 功能：保存数据到Flash
 * 参数�?
 *         para：数据指�?   len：数据长�?
 * 返回�?
 *         true：成�?  false：失�?
 * 描述�?
********************************************/
u8 SaveData_C(u8 flash_page,u8 *para,u8 len)
{
    u8 *data;
    u16 crc=0,crc2;
    CPU_SR_ALLOC();
    data=(u8*) mymalloc(&malloc_dev,len+2);
    mymemcpy(data,para,len);
    crc=CRC_16(data,len);
    data[len]=(u8)crc;
    data[len+1]=(u8)(crc>>8);
    OS_CRITICAL_ENTER();    //进入临界�?
    P_WriteFlashWord(flash_page,0,(len+2)/2,(u16*)data);    //存入FLash
    delay_ms(100);
    P_ReadFlashNBtye(flash_page,0,(u16*)data,(len+2)/2);     //读取数据
    OS_CRITICAL_EXIT();     //退出临界区
    crc=CRC_16(data,len);
    crc2=data[len+1];
    crc2=(crc2<<8)+data[len];
    myfree(&malloc_dev,data);
    if(crc!=crc2)
    {
        return false;
    }
    return true;
}
/************************
 * 功能：读取保存的数据
 * 参数�?
 *         para:数据指针
 *         len:数据长度
 * 返回�?
 *         true：成�?   false：失�?
 * *********************/
u8 ReadData(uint8_t flash_page,u8 *para,u8 len)
{
    u16 crc=0,crc2=0;
    u8 *data;
    CPU_SR_ALLOC();
    data=(u8*) mymalloc(&malloc_dev,len+2);
    OS_CRITICAL_ENTER();    //进入临界�?
    P_ReadFlashNBtye(flash_page,0,(u16*)data,(len+2)/2);     //读取数据
    OS_CRITICAL_EXIT();     //退出临界区
    crc=CRC_16(data,len);
    crc2=data[len+1];
    crc2=(crc2<<8)+data[len];
    if(crc==crc2)
    {
        mymemcpy(para,data,len);
        myfree(&malloc_dev,data);
        return true;
    }
    else
    {
        myfree(&malloc_dev,data);
        return false;
    }
}
