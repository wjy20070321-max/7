#include "app_control.h"
#include "bsp_motor.h"    
#include "bsp_encoder.h"
#include "bsp_sensor.h" 
#include "bsp_sa100.h"
#include "tim.h"

/* ------------------- 新增的 Debug 测试变量 ------------------- */
uint8_t Debug_Mode_Enable = 0; // 设为 1 进入测试模式，全部硬件正常后改为 0
float test_pwm_L = 0;          
float test_pwm_R = 0;          
int16_t test_enc_L = 0;        
int16_t test_enc_R = 0;        
float test_angle = 0;          
float test_track_err = 0;      
uint8_t test_is_B_point = 0;   

// ?【新增测试变量】专门用来观测串口屏通讯
uint8_t test_screen_task = 0;   

/* ----------------- 新增的 VOFA+ 转速观测变量 ----------------- */
float Actual_Speed_L = 0.0f;   // 左轮实际转速 (RPM)
float Actual_Speed_R = 0.0f;   // 右轮实际转速 (RPM)
float Target_Speed_L = 0.0f;   // 左轮目标转速 (RPM)
float Target_Speed_R = 0.0f;   // 右轮目标转速 (RPM)

/* ========================================================= */
/* 【核心】PID 控制器参数全局变量                             */
/* ========================================================= */
// 1. 直立环参数 (PD 控制)
float Balance_Kp = 50.0f;            // 比例系数
float Balance_Kd = 1.5f;             // 微分系数
float Mechanical_Middle = 145.0f;    // 机械中值：小车刚好平衡时的真实物理角度

// 2. 速度环参数 (PI 控制) - 倒立摆串级专用
// ??【核心修复】速度环必须为负数反馈，否则会导致前冲死循环！
float Velocity_Kp = -0.2f;           
float Velocity_Ki = -0.01f;

// 3. 循迹环参数 (PD 控制)
float Track_Kp = 30.0f;
float Track_Kd = 10.0f;
float Track_Base_Speed = 350.0f;     
float last_track_error = 0.0f;
/* ========================================================= */

/* 全局控制变量 */
volatile Target_Task_e Current_Task = TASK_IDLE;
volatile uint32_t Task_Timer_Ms = 0;

float Balance_PWM = 0, Velocity_PWM = 0, Track_PWM = 0; 
float Left_Motor_Out = 0, Right_Motor_Out = 0;

/* --- 底层支撑函数 --- */
void Motor_Output_Limit(float *left, float *right) {
    if(*left > 999.0f) *left = 999.0f;
    if(*left < -999.0f) *left = -999.0f;
    if(*right > 999.0f) *right = 999.0f;
    if(*right < -999.0f) *right = -999.0f;
}

uint8_t Is_Reach_B_Point(void) {
    uint8_t black_sum = 0;
    if(HAL_GPIO_ReadPin(TRACK3_PORT, TRACK3_PIN) == GPIO_PIN_RESET) black_sum++;
    if(HAL_GPIO_ReadPin(TRACK4_PORT, TRACK4_PIN) == GPIO_PIN_RESET) black_sum++;
    if(HAL_GPIO_ReadPin(TRACK5_PORT, TRACK5_PIN) == GPIO_PIN_RESET) black_sum++;
    if(HAL_GPIO_ReadPin(TRACK6_PORT, TRACK6_PIN) == GPIO_PIN_RESET) black_sum++;
    
    if (black_sum >= 3) {
        return 1; 
    }
    return 0; 
}

/* ========================================================= */
/* 【核心算法】三大控制环计算函数                            */
/* ========================================================= */

// 1. 循迹 PD 计算函数
float Calculate_Track_PWM(void) {
    float current_error = Sensor_Get_Track_Error(); 
    float delta_error = current_error - last_track_error;
    float turn_adjust = (current_error * Track_Kp) + (delta_error * Track_Kd);
    
    last_track_error = current_error;
    return turn_adjust;
}

// 2. 直立 PD 计算函数
float Calculate_Balance_PWM(float current_angle, float target_angle) {
    static float last_angle = 145.0f;
    
    float error = current_angle - target_angle;      
    float slope = current_angle - last_angle;        
    last_angle = current_angle;
    
    float balance_out = (error * Balance_Kp) + (slope * Balance_Kd);
    return balance_out;
}

// 3. 速度 PI 计算函数 (升级版：倒立摆串级模式)
float Calculate_Velocity_PWM(float target_speed, float actual_l, float actual_r) {
    static float velocity_integral = 0;
    
    float current_speed = (actual_l + actual_r) / 2.0f;
    float error = target_speed - current_speed;
    
    // ??【核心修复】将这里的积分判定区间同步修改为 105 ~ 185 度
    if (Pendulum_Angle > 105.0f && Pendulum_Angle < 185.0f) {
        velocity_integral += error;
        if(velocity_integral > 50.0f)  velocity_integral = 50.0f;  
        if(velocity_integral < -50.0f) velocity_integral = -50.0f;
    } else {
        velocity_integral = 0; 
    }
    
    float angle_adjustment = (error * Velocity_Kp) + (velocity_integral * Velocity_Ki);
    return angle_adjustment;
}
/* ========================================================= */

void Reset_Control_Variables(void) {
    Task_Timer_Ms = 0;
    Balance_PWM = 0;
    Velocity_PWM = 0;
    Track_PWM = 0;
    last_track_error = 0;
}

void Control_Task_Init(void) {
    Sensor_Init(); 
    Reset_Control_Variables();
}

/* ------------------------------ */

void Control_Task_Loop_5ms(void)
{
    /* ========================================================= */
    /* Debug 硬件测试拦截器                                      */
    /* ========================================================= */
    if (Debug_Mode_Enable == 1)
    {
        test_screen_task = (uint8_t)Current_Task;
        test_angle = Pendulum_Angle;
        
        test_enc_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        test_enc_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        
        Actual_Speed_L = (float)test_enc_L * 12000.0f / 1560.0f;
        Actual_Speed_R = (float)test_enc_R * 12000.0f / 1560.0f;
        
        Target_Speed_L = 150.0f;
        Target_Speed_R = 150.0f;
        
        test_track_err = Sensor_Get_Track_Error(); 
        test_is_B_point = Is_Reach_B_Point();

        Motor_Set_Speed((int16_t)test_pwm_L, (int16_t)test_pwm_R);
        return; 
    }
    /* ========================================================= */

    // 1. 获取当前 5ms 内的原始脉冲数并立刻清零
    int16_t case_enc_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t case_enc_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    
    // 2. 计算实际转速
    Actual_Speed_L = (float)case_enc_L * 12000.0f / 1560.0f;
    Actual_Speed_R = (float)case_enc_R * 12000.0f / 1560.0f;
    
    switch(Current_Task)
    {
        case TASK_IDLE:
            Motor_Set_Speed(0, 0); 
            Reset_Control_Variables();
            return;

        case TASK_1_PURE_TRACK:
            if (Is_Reach_B_Point()) {
                Current_Task = TASK_IDLE; 
            }
            Velocity_PWM = Track_Base_Speed; 
            Track_PWM = Calculate_Track_PWM(); 
            
            Left_Motor_Out  = Velocity_PWM + Track_PWM;
            Right_Motor_Out = Velocity_PWM - Track_PWM;
            break;

        case TASK_2_STAY_BALANCE:
            if (Pendulum_Angle > 105.0f && Pendulum_Angle < 185.0f) 
            {
                float Angle_Delta = Calculate_Velocity_PWM(0.0f, Actual_Speed_L, Actual_Speed_R);
                float Dynamic_Target_Angle = Mechanical_Middle + Angle_Delta;
                
                Balance_PWM  = Calculate_Balance_PWM(Pendulum_Angle, Dynamic_Target_Angle);
                Track_PWM    = 0;
                
                Left_Motor_Out  = Balance_PWM;
                Right_Motor_Out = Balance_PWM;
            } 
            else 
            {
                Left_Motor_Out  = 0; 
                Right_Motor_Out = 0;
            }
            break;

        case TASK_3_TRACK_BALANCE:
            Task_Timer_Ms += 5;
            if (Pendulum_Angle > 105.0f && Pendulum_Angle < 185.0f) 
            {
                float Angle_Delta = Calculate_Velocity_PWM(Track_Base_Speed, Actual_Speed_L, Actual_Speed_R);
                float Dynamic_Target_Angle = Mechanical_Middle + Angle_Delta;
                
                Balance_PWM  = Calculate_Balance_PWM(Pendulum_Angle, Dynamic_Target_Angle);
                Track_PWM    = Calculate_Track_PWM(); 
                
                Left_Motor_Out  = Balance_PWM + Track_PWM;
                Right_Motor_Out = Balance_PWM - Track_PWM;
                
                if (Task_Timer_Ms > 1000 && Is_Reach_B_Point()) {
                    Current_Task = TASK_IDLE; 
                }
            } 
            else 
            {
                Left_Motor_Out = 0; 
                Right_Motor_Out = 0;
            }
            break;

        case TASK_4_SPIN_BALANCE:
            if (Pendulum_Angle > 105.0f && Pendulum_Angle < 185.0f) {
                float Angle_Delta = Calculate_Velocity_PWM(0.0f, Actual_Speed_L, Actual_Speed_R);
                float Dynamic_Target_Angle = Mechanical_Middle + Angle_Delta;
                
                Balance_PWM  = Calculate_Balance_PWM(Pendulum_Angle, Dynamic_Target_Angle);
                float Spin_Value = 200.0f; 
                
                Left_Motor_Out  = Balance_PWM + Spin_Value;
                Right_Motor_Out = Balance_PWM - Spin_Value;
            } else {
                Left_Motor_Out = 0; 
                Right_Motor_Out = 0;
            }
            break;
    }

    // 硬件限幅并输出 
    Motor_Output_Limit(&Left_Motor_Out, &Right_Motor_Out);
    Motor_Set_Speed((int16_t)Left_Motor_Out, (int16_t)Right_Motor_Out); 
}
