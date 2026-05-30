#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* =========================================================
 * 任务枚举
 * ========================================================= */
typedef enum
{
    TASK_IDLE = 0,
    TASK_1_PURE_TRACK = 1,
    TASK_2_STAY_BALANCE = 2,
    TASK_3_TRACK_BALANCE = 3,
    TASK_4_SPIN_BALANCE = 4
} Target_Task_e;

/* =========================================================
 * 任务状态
 * ========================================================= */
extern volatile Target_Task_e Current_Task;
extern volatile uint32_t Task_Timer_Ms;

/* =========================================================
 * Debug / Motor Test
 * ========================================================= */

/*
 * Debug_Mode_Enable = 1:
 * 只用于在线观察变量、在线修改 PID。
 * 注意：这个模式不会拦截 PID，任务仍然正常运行。
 */
extern volatile uint8_t Debug_Mode_Enable;

/*
 * Motor_Test_Mode_Enable = 1:
 * 手动 PWM 测电机、测编码器方向。
 * 注意：这个模式会绕过 PID，直接输出 test_pwm_L/R。
 */
extern volatile uint8_t Motor_Test_Mode_Enable;

extern volatile float test_pwm_L;
extern volatile float test_pwm_R;
extern volatile int16_t test_enc_L;
extern volatile int16_t test_enc_R;
extern volatile float test_angle;
extern volatile float test_track_err;
extern volatile uint8_t test_is_B_point;
extern volatile uint8_t test_screen_task;
extern volatile uint8_t test_track_raw_bits;

/* =========================================================
 * PID / 调参变量
 * ========================================================= */

/* 直立环 */
extern volatile float Balance_Kp;
extern volatile float Balance_Kd;
extern volatile float Mechanical_Middle;

/* 速度环 */
extern volatile float Velocity_Kp;
extern volatile float Velocity_Ki;

/* 原地位置环 */
extern volatile float Position_Kp;
extern volatile float Position_Kd;

/* 循迹环 */
extern volatile float Track_Kp;
extern volatile float Track_Kd;
extern volatile float Track_Pure_Base_PWM;
extern volatile float Track_Balance_Target_RPM;
extern volatile float Track_Base_Speed;

/*
 * 循迹方向：
 *  1.0f  ：默认方向
 * -1.0f  ：反向
 *
 * 这个可以在 Live Expressions 里直接改。
 */
extern volatile float Track_Turn_Direction;

/* =========================================================
 * VOFA / HMI 观测变量
 * ========================================================= */
extern float Actual_Speed_L;
extern float Actual_Speed_R;
extern float Target_Speed_L;
extern float Target_Speed_R;

extern float Balance_PWM;
extern float Velocity_PWM;
extern float Track_PWM;
extern float Left_Motor_Out;
extern float Right_Motor_Out;

/* =========================================================
 * 函数声明
 * ========================================================= */
void Control_Task_Init(void);
void Control_Task_Loop_5ms(void);
void Reset_Control_Variables(void);

void Motor_Output_Limit(float *left, float *right);

uint8_t Is_Reach_B_Point(void);

float Calculate_Track_PWM(void);
float Calculate_Balance_PWM(float current_angle, float target_angle);
float Calculate_Velocity_PWM(float target_speed, float actual_l, float actual_r);

#ifdef __cplusplus
}
#endif

#endif
