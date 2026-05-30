#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum {
    TASK_IDLE = 0,
    TASK_1_PURE_TRACK = 1,
    TASK_2_STAY_BALANCE = 2,
    TASK_3_TRACK_BALANCE = 3,
    TASK_4_SPIN_BALANCE = 4
} Target_Task_e;

typedef enum {
    CTRL_PHASE_WAIT_UPRIGHT = 0,
    CTRL_PHASE_STABILIZE = 1,
    CTRL_PHASE_RUN = 2,
    CTRL_PHASE_STOPPED = 3
} Control_Phase_e;

extern volatile Target_Task_e Current_Task;
extern volatile Control_Phase_e Control_Phase;
extern volatile uint32_t Task_Timer_Ms;

extern volatile uint8_t Debug_Mode_Enable;
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

extern volatile float Balance_Kp;
extern volatile float Balance_Kd;
extern volatile float Mechanical_Middle;
extern volatile float Velocity_Kp;
extern volatile float Velocity_Ki;
extern volatile float Velocity_Direction;
extern volatile float Position_Kp;
extern volatile float Position_Kd;
extern volatile float Track_Kp;
extern volatile float Track_Kd;
extern volatile float Track_Pure_Base_PWM;
extern volatile float Track_Balance_Target_RPM;
extern volatile float Track_Base_Speed;
extern volatile float Track_Turn_Direction;
extern volatile float Balance_Start_Duty;
extern volatile float Track_Ramp_RPM_Per_S;
extern volatile float Angle_Filter_Alpha;
extern volatile float Deadzone_PWM;

extern float Actual_Speed_L;
extern float Actual_Speed_R;
extern float Target_Speed_L;
extern float Target_Speed_R;
extern float Balance_PWM;
extern float Velocity_PWM;
extern float Track_PWM;
extern float Left_Motor_Out;
extern float Right_Motor_Out;
extern float Dynamic_Target_Angle;
extern float Position_Sum_Pulse;
extern float Current_Track_Error;
extern float Forward_Target_RPM;

void Control_Task_Init(void);
void Control_Task_Loop_5ms(void);
void Reset_Control_Variables(void);
void Motor_Output_Limit(float *left, float *right);
uint8_t Is_Reach_B_Point(void);
float Calculate_Track_PWM(void);
float Calculate_Balance_PWM(float current_angle, float target_angle);
float Calculate_Velocity_Angle(float target_speed, float actual_l, float actual_r);

#ifdef __cplusplus
}
#endif

#endif
