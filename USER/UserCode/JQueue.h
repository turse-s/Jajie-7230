#ifndef _JQUEUE__H
#define _JQUEUE__H
#include "stdint.h"
typedef enum q_err
{
    Q_ERR_None                  = 0u,
    Q_ERR_Cache                 = 0x0001u,      //缓存区大小和队列大小不相�?
    Q_ERR_QSize_None            = 0x0002u,      //队列大小�?
    Q_ERR_QDataTypeSize_None    = 0x0003u,      //队列数据类型长度�?
    Q_ERR_QDataFull             = 0x0004u,      //队列�?
    Q_ERR_QDataNone             = 0x0005        //队列为空
} Q_ERR;
/*************************************************
 * 队列模式
 * **********************************************/
#define    JQ_LOCK_FIFO_QUEUE       (uint8_t)0x01       //队列锁定模式，如果队列存满。新数据不会存入队列。锁定队列中的数据不丢失
#define    JQ_NO_LOCK_FIFO_QUEUE    (uint8_t)0x02       //不锁定FIFO队列。队列存满，将新的数据保存进队尾，队首的数据将被抛弃        
typedef struct
{
    uint8_t *cache;              //队列空间
    int Queue_Size;           //队列长度
    int Deposit_Pointer;      //队列数据保存指针
    int Take_Pointer;         //队列数据取出指针
    uint8_t Queue_Mode;       //队列模式       FIFO锁定，不锁定
    int SaveQueueSize;        //队列存入长度
    int DataTypeSize;         //队列中数据类型的长度
} JQueue_t;
void J_Create_Queue(JQueue_t *queue_c, int size, void *cache, int type_size, uint8_t mode, Q_ERR *err);     //创建队列
void J_Enqueue(JQueue_t *queue_c, void *data, Q_ERR *err);                                                  //将数据存入FIFO队列
void J_Dequeue(JQueue_t *queue_c, void *data, Q_ERR *err);                                                  //在FIFO队列中取出一个数�?
void J_GetAllData(JQueue_t *queue_c, void *data, Q_ERR *err);                                               //查看FIFO队列中的所有数�?
int J_GetQueueDataLen(JQueue_t *queue_t);                                                                   //获取FIFO队列中数据的长度
void J_EmptyData(JQueue_t *queue_t);                                                                        //清空FIFO队列中的数据
#endif
