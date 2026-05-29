#ifndef __BSP_HMI_H
#define __BSP_HMI_H

#include "main.h"

/* 串口屏指令帧解析结构体 */
typedef struct {
    uint8_t rx_buf[64];
    uint8_t rx_index;
    uint8_t cmd_ready;
} HMI_t;

/* 声明全局变量 */
extern HMI_t hmi;

/* 函数声明 */
void HMI_Init(void);
void HMI_Receive_Byte(uint8_t byte);
void HMI_Process_Command(void);
void HMI_Send_Data(float angle, float speed);

#endif /* __BSP_HMI_H */
