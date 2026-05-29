#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "main.h"

/* 任务状态机枚举 */
typedef enum {
    TASK_IDLE = 0,          /* 空闲/停止状态 */
    TASK_1_PURE_TRACK,     /* 任务1：纯寻迹到 B 点停 */
    TASK_2_STAY_BALANCE,   /* 任务2：原地起摆并直立 10s */
    TASK_3_TRACK_BALANCE,  /* 任务3：起摆直立 + 寻迹到 B 点停 */
    TASK_4_SPIN_BALANCE    /* 任务4：原地旋转 + 维持直立 */
} Target_Task_e;

extern volatile Target_Task_e Current_Task;
extern volatile uint32_t Task_Timer_Ms;

/* VOFA/HMI 调试变量 */
extern float Actual_Speed_L;
extern float Actual_Speed_R;
extern float Target_Speed_L;
extern float Target_Speed_R;
extern float Left_Motor_Out;
extern float Right_Motor_Out;

void Control_Task_Init(void);
void Control_Task_Loop_5ms(void);
void Reset_Control_Variables(void);

#endif /* __APP_CONTROL_H */
