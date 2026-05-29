#include "app_control.h"

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_sensor.h"
#include "bsp_sa100.h"
#include "tim.h"

/* =========================================================
 * 控制周期和硬件参数
 * ========================================================= */
#define CTRL_PERIOD_MS              5U
#define CTRL_PERIOD_MS_F            5.0f
#define ENCODER_PPR                 1560.0f
#define ANGLE_SAFE_MIN              105.0f
#define ANGLE_SAFE_MAX              185.0f

/* A/B 黑圆点是直径 4cm，黑线宽 1.8cm。先走出 A 点后再允许识别 B 点。 */
#define B_POINT_ACTIVE_COUNT_MIN    5U
#define B_POINT_CENTER_COUNT_MIN    3U
#define B_POINT_DEBOUNCE_COUNT      3U
#define B_POINT_MIN_TRAVEL_PULSE    1200.0f
#define B_POINT_MIN_TIME_MS         800U

/* =========================================================
 * Debug 测试变量
 * ========================================================= */
uint8_t Debug_Mode_Enable = 0;
float test_pwm_L = 0.0f;
float test_pwm_R = 0.0f;
int16_t test_enc_L = 0;
int16_t test_enc_R = 0;
float test_angle = 0.0f;
float test_track_err = 0.0f;
uint8_t test_is_B_point = 0;
uint8_t test_screen_task = 0;

/* VOFA+ 转速观测变量 */
float Actual_Speed_L = 0.0f;
float Actual_Speed_R = 0.0f;
float Target_Speed_L = 0.0f;
float Target_Speed_R = 0.0f;

/* =========================================================
 * PID 参数：先用保守值，最终必须实车调参
 * ========================================================= */
float Balance_Kp = 50.0f;
float Balance_Kd = 1.5f;
float Mechanical_Middle = 145.0f;

/* 速度环用于修正目标角度。若车越控越冲，先整体反号 Velocity_Kp/Ki。 */
float Velocity_Kp = -0.2f;
float Velocity_Ki = -0.01f;

/* 原地平衡位置环：用于压住 A 点附近漂移。若离 A 越来越远，整体反号。 */
float Position_Kp = -0.0020f;
float Position_Kd = -0.0005f;

float Track_Kp = 30.0f;
float Track_Kd = 10.0f;
float Track_Base_Speed = 350.0f;
float last_track_error = 0.0f;

/* =========================================================
 * 全局控制变量
 * ========================================================= */
volatile Target_Task_e Current_Task = TASK_IDLE;
volatile uint32_t Task_Timer_Ms = 0;

float Balance_PWM = 0.0f;
float Velocity_PWM = 0.0f;
float Track_PWM = 0.0f;
float Left_Motor_Out = 0.0f;
float Right_Motor_Out = 0.0f;

/* 内部状态变量 */
static float velocity_integral = 0.0f;
static float balance_last_angle = 145.0f;
static float position_sum_pulse = 0.0f;
static float last_position_error = 0.0f;
static float travel_pulse_abs = 0.0f;
static uint8_t b_point_debounce = 0U;

static float Limit_Float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static uint8_t Is_Angle_Safe(void)
{
    return (Pendulum_Angle > ANGLE_SAFE_MIN && Pendulum_Angle < ANGLE_SAFE_MAX) ? 1U : 0U;
}

static void Clear_Dynamic_Loops(void)
{
    velocity_integral = 0.0f;
    balance_last_angle = Pendulum_Angle;
    last_track_error = 0.0f;
    last_position_error = 0.0f;
    b_point_debounce = 0U;
}

void Motor_Output_Limit(float *left, float *right)
{
    *left = Limit_Float(*left, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
    *right = Limit_Float(*right, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
}

/**
 * @brief B 点识别。
 *        统一使用 bsp_sensor.h 里的 TRACK_LINE_ACTIVE_LEVEL，避免循迹和停车极性矛盾。
 */
uint8_t Is_Reach_B_Point(void)
{
    uint8_t all_active = Sensor_Count_Line_Active();
    uint8_t center_active = Sensor_Count_Center_Line_Active();

    if (travel_pulse_abs < B_POINT_MIN_TRAVEL_PULSE) {
        b_point_debounce = 0U;
        return 0U;
    }

    if (Task_Timer_Ms < B_POINT_MIN_TIME_MS) {
        b_point_debounce = 0U;
        return 0U;
    }

    if (all_active >= B_POINT_ACTIVE_COUNT_MIN || center_active >= B_POINT_CENTER_COUNT_MIN) {
        if (b_point_debounce < B_POINT_DEBOUNCE_COUNT) {
            b_point_debounce++;
        }
    } else {
        b_point_debounce = 0U;
    }

    return (b_point_debounce >= B_POINT_DEBOUNCE_COUNT) ? 1U : 0U;
}

/* =========================================================
 * 三大控制环计算函数
 * ========================================================= */
float Calculate_Track_PWM(void)
{
    float current_error = Sensor_Get_Track_Error();
    float delta_error = current_error - last_track_error;
    float turn_adjust = (current_error * Track_Kp) + (delta_error * Track_Kd);

    last_track_error = current_error;
    return turn_adjust;
}

float Calculate_Balance_PWM(float current_angle, float target_angle)
{
    float error = current_angle - target_angle;
    float slope = current_angle - balance_last_angle;

    balance_last_angle = current_angle;

    return (error * Balance_Kp) + (slope * Balance_Kd);
}

float Calculate_Velocity_PWM(float target_speed, float actual_l, float actual_r)
{
    float current_speed = (actual_l + actual_r) / 2.0f;
    float error = target_speed - current_speed;

    if (Is_Angle_Safe()) {
        velocity_integral += error;
        velocity_integral = Limit_Float(velocity_integral, -50.0f, 50.0f);
    } else {
        velocity_integral = 0.0f;
    }

    return (error * Velocity_Kp) + (velocity_integral * Velocity_Ki);
}

static float Calculate_Position_Angle_Compensation(float delta_pulse_avg)
{
    float position_error;
    float position_delta;
    float angle_comp;

    position_sum_pulse += delta_pulse_avg;
    position_sum_pulse = Limit_Float(position_sum_pulse, -6000.0f, 6000.0f);

    position_error = 0.0f - position_sum_pulse;
    position_delta = position_error - last_position_error;
    last_position_error = position_error;

    angle_comp = (position_error * Position_Kp) + (position_delta * Position_Kd);

    /* 原地位置环只做小角度慢修正，防止把直立环顶飞。 */
    return Limit_Float(angle_comp, -8.0f, 8.0f);
}

void Reset_Control_Variables(void)
{
    Task_Timer_Ms = 0U;

    Balance_PWM = 0.0f;
    Velocity_PWM = 0.0f;
    Track_PWM = 0.0f;
    Left_Motor_Out = 0.0f;
    Right_Motor_Out = 0.0f;

    Target_Speed_L = 0.0f;
    Target_Speed_R = 0.0f;

    velocity_integral = 0.0f;
    balance_last_angle = Pendulum_Angle;
    last_track_error = 0.0f;
    position_sum_pulse = 0.0f;
    last_position_error = 0.0f;
    travel_pulse_abs = 0.0f;
    b_point_debounce = 0U;
}

void Control_Task_Init(void)
{
    Sensor_Init();
    Reset_Control_Variables();
}

void Control_Task_Loop_5ms(void)
{
    int16_t case_enc_L;
    int16_t case_enc_R;
    float delta_pulse_avg;

    /* Debug 硬件测试拦截器 */
    if (Debug_Mode_Enable == 1U) {
        test_screen_task = (uint8_t)Current_Task;
        test_angle = Pendulum_Angle;

        test_enc_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        test_enc_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        __HAL_TIM_SET_COUNTER(&htim4, 0);

        Actual_Speed_L = ((float)test_enc_L * 60000.0f) / (CTRL_PERIOD_MS_F * ENCODER_PPR);
        Actual_Speed_R = ((float)test_enc_R * 60000.0f) / (CTRL_PERIOD_MS_F * ENCODER_PPR);

        Target_Speed_L = 150.0f;
        Target_Speed_R = 150.0f;

        test_track_err = Sensor_Get_Track_Error();
        test_is_B_point = Is_Reach_B_Point();

        Motor_Set_Speed((int16_t)test_pwm_L, (int16_t)test_pwm_R);
        return;
    }

    /* 获取当前控制周期内编码器脉冲并清零 */
    case_enc_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    case_enc_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    delta_pulse_avg = ((float)case_enc_L + (float)case_enc_R) / 2.0f;
    if (delta_pulse_avg >= 0.0f) {
        travel_pulse_abs += delta_pulse_avg;
    } else {
        travel_pulse_abs -= delta_pulse_avg;
    }

    Actual_Speed_L = ((float)case_enc_L * 60000.0f) / (CTRL_PERIOD_MS_F * ENCODER_PPR);
    Actual_Speed_R = ((float)case_enc_R * 60000.0f) / (CTRL_PERIOD_MS_F * ENCODER_PPR);

    if (Current_Task == TASK_IDLE) {
        Motor_Set_Speed(0, 0);
        Reset_Control_Variables();
        return;
    }

    Task_Timer_Ms += CTRL_PERIOD_MS;

    switch (Current_Task) {
    case TASK_1_PURE_TRACK:
        Target_Speed_L = Track_Base_Speed;
        Target_Speed_R = Track_Base_Speed;

        if (Is_Reach_B_Point()) {
            Current_Task = TASK_IDLE;
            Motor_Set_Speed(0, 0);
            Reset_Control_Variables();
            return;
        }

        Velocity_PWM = Track_Base_Speed;
        Track_PWM = Calculate_Track_PWM();
        Left_Motor_Out = Velocity_PWM + Track_PWM;
        Right_Motor_Out = Velocity_PWM - Track_PWM;
        break;

    case TASK_2_STAY_BALANCE:
        Target_Speed_L = 0.0f;
        Target_Speed_R = 0.0f;

        if (Is_Angle_Safe()) {
            float angle_delta = Calculate_Velocity_PWM(0.0f, Actual_Speed_L, Actual_Speed_R);
            float position_angle_comp = Calculate_Position_Angle_Compensation(delta_pulse_avg);
            float dynamic_target_angle = Mechanical_Middle + angle_delta + position_angle_comp;

            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle, dynamic_target_angle);
            Track_PWM = 0.0f;
            Left_Motor_Out = Balance_PWM;
            Right_Motor_Out = Balance_PWM;
        } else {
            Clear_Dynamic_Loops();
            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    case TASK_3_TRACK_BALANCE:
        Target_Speed_L = Track_Base_Speed;
        Target_Speed_R = Track_Base_Speed;

        if (Is_Angle_Safe()) {
            float angle_delta = Calculate_Velocity_PWM(Track_Base_Speed, Actual_Speed_L, Actual_Speed_R);
            float dynamic_target_angle = Mechanical_Middle + angle_delta;

            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle, dynamic_target_angle);
            Track_PWM = Calculate_Track_PWM();
            Left_Motor_Out = Balance_PWM + Track_PWM;
            Right_Motor_Out = Balance_PWM - Track_PWM;

            if (Is_Reach_B_Point()) {
                Current_Task = TASK_IDLE;
                Motor_Set_Speed(0, 0);
                Reset_Control_Variables();
                return;
            }
        } else {
            Clear_Dynamic_Loops();
            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    case TASK_4_SPIN_BALANCE:
        Target_Speed_L = 0.0f;
        Target_Speed_R = 0.0f;

        if (Is_Angle_Safe()) {
            float angle_delta = Calculate_Velocity_PWM(0.0f, Actual_Speed_L, Actual_Speed_R);
            float dynamic_target_angle = Mechanical_Middle + angle_delta;
            float spin_value = 200.0f;

            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle, dynamic_target_angle);
            Left_Motor_Out = Balance_PWM + spin_value;
            Right_Motor_Out = Balance_PWM - spin_value;
        } else {
            Clear_Dynamic_Loops();
            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    default:
        Current_Task = TASK_IDLE;
        Motor_Set_Speed(0, 0);
        Reset_Control_Variables();
        return;
    }

    Motor_Output_Limit(&Left_Motor_Out, &Right_Motor_Out);
    Motor_Set_Speed((int16_t)Left_Motor_Out, (int16_t)Right_Motor_Out);
}
