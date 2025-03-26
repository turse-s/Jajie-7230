/******************************************************************************
 * Author:  huiaove
 * Email:   huiaove@126.com
 * Date:    2020 12.20
 * Address: TianJin
 * Version: V1.0.1
 * 修改记录�?
 *             2020 12.20    张亚�?   初始版本               版本�?V1.0.0
 *             2021 02.16    张亚�?   修改队列存取指针问题   版本�?V1.0.1
 * 功能说明：FIFO队列，支持创建任意数据类型队列。队列FIFO有两种模式，锁定模式和非锁定模式
 * ***************************************************************************/
#include "JQueue.h"
/****************************************
 * 功能：创建消息队�?
 * 参数�?
 *         *queue_c:FIFO队列结构�?
 *         size:队列长度
 *         *cache:队列缓存空间
 *         type_szie:队列数据类型长度
 *         mode:队列模式
 *         *err:错误�?
 * 返回：无
 * 修改记录�?
 *             张亚�?  2020 12.30    创建函数
 * *************************************/
void J_Create_Queue(JQueue_t *queue_c, int size, void *cache, int type_size, uint8_t mode, Q_ERR *err)
{
    if (size == 0)
    {
        *err = Q_ERR_QSize_None;
        return;
    }
    if (type_size == 0)
    {
        *err = Q_ERR_QDataTypeSize_None;
        return;
    }
    queue_c->cache = (uint8_t *)cache;
//    if ((sizeof(queue_c->cache) / type_size) != size)
//    {
//        *err = Q_ERR_Cache;
//        return;
//    }
    queue_c->Queue_Size=size;
    queue_c->DataTypeSize = type_size/8;
    queue_c->Deposit_Pointer = 0;
    queue_c->Take_Pointer = 0;
    queue_c->SaveQueueSize = 0;
    queue_c->Queue_Mode = mode;
}
/****************************************
 * 功能：将数据存入FIFO队列
 * 参数�?
 *         *queue_c:FIFO队列结构�?
 *         *data:要存入FIFO队列中的数据
 *         *err:错误�?
 * 返回：无
 * 修改记录�?
 *             张亚�?   2020 12.20    创建存入函数
 * *************************************/
void J_Enqueue(JQueue_t *queue_c, void *data, Q_ERR *err)
{
    int i = 0;
    if (queue_c->SaveQueueSize >= queue_c->Queue_Size)
    {
        if (queue_c->Queue_Mode == JQ_LOCK_FIFO_QUEUE)
        {
            *err = Q_ERR_QDataFull;
            return;
        }
        else
        {
            for (i = 0; i < queue_c->DataTypeSize; i++)
            {
                *(queue_c->cache + (queue_c->Deposit_Pointer * queue_c->DataTypeSize) + i) = *((uint8_t *)data + i);
            }
            queue_c->Deposit_Pointer++;
            queue_c->Deposit_Pointer %= queue_c->Queue_Size;
            queue_c->Take_Pointer++;
            queue_c->Take_Pointer %= queue_c->Queue_Size;
        }
    }
    else
    {
        for (i = 0; i < queue_c->DataTypeSize; i++)
        {
            *(queue_c->cache + (queue_c->Deposit_Pointer * queue_c->DataTypeSize) + i) = *((uint8_t *)data + i);
        }
        queue_c->Deposit_Pointer++;
        queue_c->Deposit_Pointer %= queue_c->Queue_Size;
        queue_c->SaveQueueSize++;
    }
    *err = Q_ERR_None;
}
/****************************************
 * 功能：在队列中取出一个数�?
 * 参数�?
 *         *queue_c:FIFO队列结构�?
 *         *data:存放取出数据的指�?
 *         *err:错误�?
 * 返回�?
 * 修改记录�?
 *             张亚�?   2020 12.20    创建函数
 * *************************************/
void J_Dequeue(JQueue_t *queue_c, void *data, Q_ERR *err)
{
    int i = 0;
    if (queue_c->SaveQueueSize == 0)
    {
        *err = Q_ERR_QDataNone;
        return;
    }
    for (i = 0; i < queue_c->DataTypeSize; i++)
    {
        *((uint8_t *)data + i) = *(queue_c->cache + (queue_c->Take_Pointer * queue_c->DataTypeSize) + i);
    }
    *err=Q_ERR_None;
    queue_c->Take_Pointer++;
    queue_c->Take_Pointer %= queue_c->Queue_Size;
    queue_c->SaveQueueSize--;
}
/***********************************
 * 功能：查看队列中的所有数据，不会改变FIFO队列数据
 * 参数�?
 *         *queue_c:FIFO队列结构�?
 *         *data:接收队列数据的指�?
 *         *err: 错误�?
 * 返回：无
 * 修改记录�?
 *             张亚�?   2020 12.20    创建函数
 * ********************************/
void J_GetAllData(JQueue_t *queue_c, void *data, Q_ERR *err)
{
    int i = 0, j = 0;
    int take_pointer = queue_c->Take_Pointer;
    if (queue_c->SaveQueueSize == 0)
    {
        *err = Q_ERR_QDataNone;
        return;
    }
    for (j = 0; j < queue_c->SaveQueueSize; j++)
    {
        for (i = 0; i < queue_c->DataTypeSize; i++)
        {
            *((uint8_t *)data + i + j * queue_c->DataTypeSize) = *(queue_c->cache + take_pointer * queue_c->DataTypeSize + i);
        }
        take_pointer++;
        take_pointer %= queue_c->Queue_Size;
    }
}
/*********************************
 * 功能：获取队列中数据长度
 * 参数�?
 *         *queue_t:FIFO队列结构�?
 * 返回�?
 *         队列数据长度
 * 修改记录�?
 *             张亚�?   2020 12.20    创建函数
 * ******************************/
int J_GetQueueDataLen(JQueue_t *queue_t)
{
    return queue_t->SaveQueueSize;
}
/*********************************
 * 功能：清空FIFO队列中的所有数�?
 * 参数�?
 *         *queue_t:FIFO队列结构�?
 * 返回：无
 * 修改记录�?
 *             张亚�?   2020 12.20    创建函数
 * ******************************/
void J_EmptyData(JQueue_t *queue_t)
{
    queue_t->SaveQueueSize=0;
    queue_t->Deposit_Pointer=0;
    queue_t->Take_Pointer=0;
}
