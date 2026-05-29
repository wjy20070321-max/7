#include "bsp_hmi.h"
#include "app_control.h"
#include "usart.h"
#include <stdio.h> 

// 引入外部控制变量用于数据显示
extern float Left_Motor_Out;
extern float Right_Motor_Out;
extern float Actual_Speed_L;
extern float Pendulum_Angle;

/* 串口接收静态缓冲区 */
static uint8_t rx_buf[16];
static uint8_t rx_index = 0;

/* ========================================================= */
/* 【调整顺序】把清空缓冲区的底层函数挪到最上方，消除隐式声明错误 */
/* ========================================================= */
void HMI_Clear_Buffer(void) {
    rx_index = 0;
    for(int i=0; i<16; i++) rx_buf[i] = 0;
}

void HMI_Init(void) {
    // 此时调用 HMI_Clear_Buffer，编译器已经在上方认识它了，不再会报错！
    HMI_Clear_Buffer();
}

// 非阻塞检测：看串口是否收到了屏幕发来的 3 字节标准控制帧（7A 任务号 7B）
uint8_t HMI_Buffer_Ready(void) {
    uint8_t tmp;
    // 只要串口有数据，就塞进本地缓存
    while (HAL_UART_Receive(&huart1, &tmp, 1, 0) == HAL_OK) {
        rx_buf[rx_index++] = tmp;
        if (rx_index >= 16) rx_index = 0; // 防止越界
    }
    
    // 扫视缓冲区，寻找 0x7A 包头和 0x7B 包尾
    for (int i = 0; i < rx_index; i++) {
        if (rx_buf[i] == 0x7A && (i + 2) < rx_index && rx_buf[i + 2] == 0x7B) {
            return 1; // 抓到合法的一帧！
        }
    }
    return 0;
}

// 提取任务 ID
uint8_t Get_HMI_Cmd(void) {
    for (int i = 0; i < rx_index; i++) {
        if (rx_buf[i] == 0x7A && (i + 2) < rx_index && rx_buf[i + 2] == 0x7B) {
            return rx_buf[i + 1]; // 返回包头后面的那个任务号
        }
    }
    return 0;
}

void HMI_Send_End(void) {
    uint8_t end_cmd[3] = {0xFF, 0xFF, 0xFF};
    HAL_UART_Transmit(&huart1, end_cmd, 3, 10);
}

// 核心命令处理
void HMI_Process_Command(void)
{
    if (HMI_Buffer_Ready()) 
    {
        uint8_t cmd = Get_HMI_Cmd(); 
        Reset_Control_Variables(); // 切换任务前必须清空各个控制环的积分值，防止飞车
        
        switch(cmd)
        {
            case 0x00: Current_Task = TASK_IDLE; break;
            case 0x01: Current_Task = TASK_1_PURE_TRACK; break;
            case 0x02: Current_Task = TASK_2_STAY_BALANCE; break;
            case 0x03: Current_Task = TASK_3_TRACK_BALANCE; break;
            case 0x04: Current_Task = TASK_4_SPIN_BALANCE; break;
        }
        HMI_Clear_Buffer(); // 别忘了擦拭缓冲区
    }
}

// 定时向屏幕刷数据的函数
void HMI_Send_Data(float angle, float speed)
{
    char buf[50];
    int len;
    
    // 1. 文本框 t0 显示小车当前实时真实角度
    len = sprintf(buf, "t0.txt=\"%.1f\"", angle);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
    HMI_Send_End(); 
    
//    // 2. 文本框 t1 显示左轮真实转速
//    len = sprintf(buf, "t1.txt=\"%.0f\"", speed);
//    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
//    HMI_Send_End();
//    
//    // 3. 文本框 t2 和 t3 用来监控两轮最终的 PWM 动力输出
//    len = sprintf(buf, "t2.txt=\"%.0f\"", Left_Motor_Out);
//    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
//    HMI_Send_End();
//    
//    len = sprintf(buf, "t3.txt=\"%.0f\"", Right_Motor_Out);
//    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
//    HMI_Send_End();
}
