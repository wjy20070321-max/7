#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "main.h"

/* 任务状态机枚举 */
typedef enum {
    TASK_IDLE = 0,         // 空闲/停止状态
    TASK_1_PURE_TRACK,    // 任务1：纯寻迹到B点停（不带摆）
    TASK_2_STAY_BALANCE,  // 任务2：原地起摆并直立10s
    TASK_3_TRACK_BALANCE, // 任务3：起摆直立+寻迹到B点停
    TASK_4_SPIN_BALANCE   // 任务4：原地旋转+维持直立
} Target_Task_e;

/* 外部声明全局状态变量 */
extern volatile Target_Task_e Current_Task;
extern volatile uint32_t Task_Timer_Ms; // 用于任务时间计时（如10秒维持）

/* 函数声明 */
void Control_Task_Init(void);
void Control_Task_Loop_5ms(void);
void Reset_Control_Variables(void);

#endif
