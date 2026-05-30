#include "app_control.h"

#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_sensor.h"
#include "bsp_sa100.h"
#include "tim.h"

/* =========================================================
 * 控制周期和硬件参数
 * ========================================================= */

/*
 * 控制中断周期，单位 ms。
 * 这里写 5U 表示 Control_Task_Loop_5ms() 每 5ms 运行一次。
 * 注意：tim.c 里的 TIM10 也必须真的配置成 5ms，否则速度计算会不准。
 */
#define CTRL_PERIOD_MS              5U

/*
 * 控制周期的 float 版本。
 * 因为后面 RPM_SCALE 要参与浮点计算，所以单独定义 5.0f。
 */
#define CTRL_PERIOD_MS_F            5.0f

/*
 * 编码器一圈的脉冲数。
 * 这里 1560.0f 表示车轮/电机输出轴转一圈，编码器计数约为 1560。
 * 如果你的编码器实际线数、减速比不同，这个值要改。
 */
#define ENCODER_PPR                 1560.0f

/*
 * 编码器脉冲数换算成 RPM 的比例系数。
 *
 * 公式：
 * RPM = 单次周期内脉冲数 * 60000 / (控制周期ms * 每圈脉冲数)
 */
#define RPM_SCALE                   (60000.0f / (CTRL_PERIOD_MS_F * ENCODER_PPR))

/*
 * 编码器方向修正。
 *
 * 目标：
 * 小车向前运动时，test_enc_L 和 test_enc_R 应该同号，并且数值接近。
 *
 * 你现在现象是：
 * test_enc_L = 160 多
 * test_enc_R = 负数
 *
 * 所以先把右编码器方向反过来。
 *
 * 如果之后发现左边也反，可以把 ENC_L_DIR 改成 -1。
 */
#define ENC_L_DIR                   1
#define ENC_R_DIR                  -1

/*
 * 角度安全范围。
 *
 * 只有 Pendulum_Angle 在 125° ~ 165° 之间时，
 * 代码才认为摆杆已经接近竖直，可以开启直立 PID。
 */
#define ANGLE_SAFE_MIN              125.0f
#define ANGLE_SAFE_MAX              165.0f

/*
 * 速度环积分限幅。
 */
#define VELOCITY_INTEGRAL_LIMIT     50.0f

/*
 * 速度环输出限幅。
 * 速度环输出的是目标角度修正量，不是直接 PWM。
 */
#define VELOCITY_ANGLE_LIMIT        12.0f

/*
 * 位置环输出限幅。
 * 位置环输出的也是目标角度修正量。
 */
#define POSITION_ANGLE_LIMIT        8.0f

/*
 * B 点识别保护。
 */
#define B_POINT_DEBOUNCE_COUNT      3U
#define B_POINT_MIN_TIME_MS         800U
#define B_POINT_MIN_TRAVEL_PULSE    700.0f

#ifndef B_POINT_BLACK_MIN_COUNT
#define B_POINT_BLACK_MIN_COUNT             7U
#endif

#ifndef B_POINT_CENTER_BLACK_MIN_COUNT
#define B_POINT_CENTER_BLACK_MIN_COUNT      4U
#endif

/* =========================================================
 * Debug / Motor Test 变量
 * ========================================================= */

/*
 * Debug_Mode_Enable = 1:
 * 只用于在线观察变量、在线修改 PID。
 * 不会拦截控制环。
 */
volatile uint8_t Debug_Mode_Enable = 0U;

/*
 * Motor_Test_Mode_Enable = 1:
 * 手动 PWM 测电机。
 * 会绕过 PID，直接输出 test_pwm_L / test_pwm_R。
 */
volatile uint8_t Motor_Test_Mode_Enable = 0U;

volatile float test_pwm_L = 0.0f;
volatile float test_pwm_R = 0.0f;
volatile int16_t test_enc_L = 0;
volatile int16_t test_enc_R = 0;
volatile float test_angle = 0.0f;
volatile float test_track_err = 0.0f;
volatile uint8_t test_is_B_point = 0U;
volatile uint8_t test_screen_task = 0U;
volatile uint8_t test_track_raw_bits = 0U;

/* VOFA/HMI 观测变量 */
float Actual_Speed_L = 0.0f;
float Actual_Speed_R = 0.0f;
float Target_Speed_L = 0.0f;
float Target_Speed_R = 0.0f;

/* =========================================================
 * PID 参数：全部 volatile，方便 Live Expressions 在线改
 * ========================================================= */

/* 直立环 */
volatile float Balance_Kp = 220.0f;
volatile float Balance_Kd = 10.0f;
volatile float Mechanical_Middle = 145.29957f;

/* 速度环 */
volatile float Velocity_Kp = 0.0f;
volatile float Velocity_Ki = 0.0f;

/* 原地位置环 */
volatile float Position_Kp = 0.0f;
volatile float Position_Kd = 0.0f;

/* 循迹环 */
volatile float Track_Kp = 18.0f;
volatile float Track_Kd = 4.0f;

/*
 * 任务一纯寻迹基础 PWM。
 * 注意：如果 MOTOR_MAX_PWM 是 999，1100 会被限幅成 999。
 */
volatile float Track_Pure_Base_PWM = 1100.0f;

/* 任务三边平衡边循迹目标速度 */
volatile float Track_Balance_Target_RPM = 160.0f;

/* 兼容旧变量名 */
volatile float Track_Base_Speed = 1100.0f;

/*
 * 循迹转向方向。
 * 如果寻迹方向反了，在 Live Expressions 里改成 -1.0f。
 */
volatile float Track_Turn_Direction = 1.0f;

/* =========================================================
 * 全局控制变量
 * ========================================================= */
volatile Target_Task_e Current_Task = TASK_IDLE;
volatile uint32_t Task_Timer_Ms = 0U;

float Balance_PWM = 0.0f;
float Velocity_PWM = 0.0f;
float Track_PWM = 0.0f;
float Left_Motor_Out = 0.0f;
float Right_Motor_Out = 0.0f;

/* =========================================================
 * 内部状态变量
 * ========================================================= */
static float velocity_integral = 0.0f;
static float balance_last_angle = 145.0f;
static float position_sum_pulse = 0.0f;
static float last_position_error = 0.0f;
static float last_track_error = 0.0f;
static float travel_pulse_abs = 0.0f;
static uint8_t b_point_debounce = 0U;

/* =========================================================
 * 工具函数
 * ========================================================= */
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

static float Abs_Float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t Is_Angle_Safe(void)
{
    if (Pendulum_Angle > ANGLE_SAFE_MIN && Pendulum_Angle < ANGLE_SAFE_MAX) {
        return 1U;
    }

    return 0U;
}

static int16_t Read_Encoder_And_Clear(TIM_HandleTypeDef *htim)
{
    int16_t count;

    count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);

    return count;
}

static void Clear_Dynamic_Loops(void)
{
    velocity_integral = 0.0f;
    balance_last_angle = Pendulum_Angle;
    last_position_error = 0.0f;
    last_track_error = 0.0f;
    b_point_debounce = 0U;

    Sensor_Reset_Track_State();
}

/* =========================================================
 * 输出限幅
 * ========================================================= */
void Motor_Output_Limit(float *left, float *right)
{
    *left = Limit_Float(*left, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
    *right = Limit_Float(*right, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
}

/* =========================================================
 * B 点检测
 * ========================================================= */
uint8_t Is_Reach_B_Point(void)
{
    uint8_t all_black;
    uint8_t center_black;
    uint8_t detected;

    all_black = Sensor_Count_Black();
    center_black = Sensor_Count_Center_Black();

    if (Task_Timer_Ms < B_POINT_MIN_TIME_MS) {
        b_point_debounce = 0U;
        return 0U;
    }

    if (travel_pulse_abs < B_POINT_MIN_TRAVEL_PULSE) {
        b_point_debounce = 0U;
        return 0U;
    }

    detected = 0U;

    if (all_black >= B_POINT_BLACK_MIN_COUNT) {
        detected = 1U;
    }

    if (center_black >= B_POINT_CENTER_BLACK_MIN_COUNT) {
        detected = 1U;
    }

    if (detected) {
        if (b_point_debounce < B_POINT_DEBOUNCE_COUNT) {
            b_point_debounce++;
        }
    } else {
        b_point_debounce = 0U;
    }

    if (b_point_debounce >= B_POINT_DEBOUNCE_COUNT) {
        return 1U;
    }

    return 0U;
}

/* =========================================================
 * 循迹 PD
 * ========================================================= */
float Calculate_Track_PWM(void)
{
    float current_error;
    float delta_error;
    float turn_adjust;

    current_error = Sensor_Get_Track_Error();
    delta_error = current_error - last_track_error;

    turn_adjust = current_error * Track_Kp + delta_error * Track_Kd;

    last_track_error = current_error;

    if (Debug_Mode_Enable == 1U) {
        test_track_err = current_error;
    }

    return turn_adjust;
}

/* =========================================================
 * 直立 PD
 * ========================================================= */
float Calculate_Balance_PWM(float current_angle, float target_angle)
{
    float error;
    float slope;
    float output;

    error = current_angle - target_angle;
    slope = current_angle - balance_last_angle;

    balance_last_angle = current_angle;

    output = error * Balance_Kp + slope * Balance_Kd;

    return output;
}

/* =========================================================
 * 速度 PI：输出为目标角度修正量
 * ========================================================= */
float Calculate_Velocity_PWM(float target_speed, float actual_l, float actual_r)
{
    float current_speed;
    float error;
    float output;

    current_speed = (actual_l + actual_r) * 0.5f;
    error = target_speed - current_speed;

    if (Is_Angle_Safe()) {
        velocity_integral += error;

        velocity_integral = Limit_Float(velocity_integral,
                                        -VELOCITY_INTEGRAL_LIMIT,
                                        VELOCITY_INTEGRAL_LIMIT);
    } else {
        velocity_integral = 0.0f;
    }

    output = error * Velocity_Kp + velocity_integral * Velocity_Ki;

    return Limit_Float(output, -VELOCITY_ANGLE_LIMIT, VELOCITY_ANGLE_LIMIT);
}

/* =========================================================
 * 位置环：输出为目标角度修正量
 * ========================================================= */
static float Calculate_Position_Angle_Compensation(float delta_pulse_avg)
{
    float position_error;
    float position_delta;
    float angle_comp;

    position_sum_pulse += delta_pulse_avg;

    position_sum_pulse = Limit_Float(position_sum_pulse,
                                     -6000.0f,
                                      6000.0f);

    position_error = 0.0f - position_sum_pulse;
    position_delta = position_error - last_position_error;
    last_position_error = position_error;

    angle_comp = position_error * Position_Kp + position_delta * Position_Kd;

    return Limit_Float(angle_comp,
                       -POSITION_ANGLE_LIMIT,
                        POSITION_ANGLE_LIMIT);
}

/* =========================================================
 * 重置控制变量
 * ========================================================= */
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

    position_sum_pulse = 0.0f;
    last_position_error = 0.0f;

    last_track_error = 0.0f;
    travel_pulse_abs = 0.0f;
    b_point_debounce = 0U;

    test_is_B_point = 0U;

    Sensor_Reset_Track_State();
}

/* =========================================================
 * 初始化
 * ========================================================= */
void Control_Task_Init(void)
{
    Sensor_Init();
    Reset_Control_Variables();
}

/* =========================================================
 * 5ms 控制主循环
 * ========================================================= */
void Control_Task_Loop_5ms(void)
{
    int16_t enc_l;
    int16_t enc_r;

    float delta_pulse_avg;
    float angle_delta;
    float position_angle_comp;
    float dynamic_target_angle;
    float turn_pwm;

    uint8_t b_reached;

    b_reached = 0U;

    /*
     * Motor_Test_Mode_Enable:
     * 只用于手动 PWM 测电机、测编码器方向。
     * 开这个模式时，不跑任务、不跑 PID。
     */
    if (Motor_Test_Mode_Enable == 1U) {
        test_screen_task = (uint8_t)Current_Task;
        test_angle = Pendulum_Angle;

        /*
         * 这里已经加入编码器方向修正。
         * 所以 test_enc_L / test_enc_R 显示的是“修正后的方向”。
         */
        test_enc_L = (int16_t)(ENC_L_DIR * Read_Encoder_And_Clear(&htim3));
        test_enc_R = (int16_t)(ENC_R_DIR * Read_Encoder_And_Clear(&htim4));

        Actual_Speed_L = (float)test_enc_L * RPM_SCALE;
        Actual_Speed_R = (float)test_enc_R * RPM_SCALE;

        test_track_raw_bits = Sensor_Get_Raw_Bits();

        test_pwm_L = Limit_Float(test_pwm_L,
                                 -(float)MOTOR_MAX_PWM,
                                  (float)MOTOR_MAX_PWM);

        test_pwm_R = Limit_Float(test_pwm_R,
                                 -(float)MOTOR_MAX_PWM,
                                  (float)MOTOR_MAX_PWM);

        Motor_Set_Speed((int16_t)test_pwm_L, (int16_t)test_pwm_R);
        return;
    }

    /*
     * 正常读取编码器。
     * 这里也加入方向修正，速度环和位置环会使用修正后的 enc_l / enc_r。
     */
    enc_l = (int16_t)(ENC_L_DIR * Read_Encoder_And_Clear(&htim3));
    enc_r = (int16_t)(ENC_R_DIR * Read_Encoder_And_Clear(&htim4));

    Actual_Speed_L = (float)enc_l * RPM_SCALE;
    Actual_Speed_R = (float)enc_r * RPM_SCALE;

    delta_pulse_avg = ((float)enc_l + (float)enc_r) * 0.5f;

    travel_pulse_abs += (Abs_Float((float)enc_l) + Abs_Float((float)enc_r)) * 0.5f;

    /*
     * Debug_Mode_Enable:
     * 只更新观测变量。
     * 不控制电机，不 return，不影响 PID。
     */
    if (Debug_Mode_Enable == 1U) {
        test_screen_task = (uint8_t)Current_Task;
        test_angle = Pendulum_Angle;
        test_enc_L = enc_l;
        test_enc_R = enc_r;
        test_track_raw_bits = Sensor_Get_Raw_Bits();
    }

    /*
     * 空闲状态
     */
    if (Current_Task == TASK_IDLE) {
        Motor_Set_Speed(0, 0);
        Reset_Control_Variables();
        return;
    }

    Task_Timer_Ms += CTRL_PERIOD_MS;

    switch (Current_Task) {
    case TASK_1_PURE_TRACK:
        /*
         * 任务一：纯寻迹。
         */
        Velocity_PWM = Track_Pure_Base_PWM;
        Track_Base_Speed = Track_Pure_Base_PWM;

        Track_PWM = Calculate_Track_PWM();
        turn_pwm = Track_Turn_Direction * Track_PWM;

        Left_Motor_Out = Velocity_PWM + turn_pwm;
        Right_Motor_Out = Velocity_PWM - turn_pwm;

        Target_Speed_L = 0.0f;
        Target_Speed_R = 0.0f;

        if (Is_Reach_B_Point()) {
            b_reached = 1U;

            Current_Task = TASK_IDLE;
            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    case TASK_2_STAY_BALANCE:
        /*
         * 任务二：原地起摆/平衡。
         */
        Target_Speed_L = 0.0f;
        Target_Speed_R = 0.0f;

        if (Is_Angle_Safe()) {
            angle_delta = Calculate_Velocity_PWM(0.0f,
                                                 Actual_Speed_L,
                                                 Actual_Speed_R);

            position_angle_comp = Calculate_Position_Angle_Compensation(delta_pulse_avg);

            dynamic_target_angle = Mechanical_Middle
                                 + angle_delta
                                 + position_angle_comp;

            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle,
                                                 dynamic_target_angle);

            Left_Motor_Out = Balance_PWM;
            Right_Motor_Out = Balance_PWM;
        } else {
            Clear_Dynamic_Loops();

            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    case TASK_3_TRACK_BALANCE:
        /*
         * 任务三：边平衡边循迹。
         */
        Target_Speed_L = Track_Balance_Target_RPM;
        Target_Speed_R = Track_Balance_Target_RPM;

        if (Is_Angle_Safe()) {
            angle_delta = Calculate_Velocity_PWM(Track_Balance_Target_RPM,
                                                 Actual_Speed_L,
                                                 Actual_Speed_R);

            dynamic_target_angle = Mechanical_Middle + angle_delta;

            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle,
                                                 dynamic_target_angle);

            Track_PWM = Calculate_Track_PWM();
            turn_pwm = Track_Turn_Direction * Track_PWM;

            Left_Motor_Out = Balance_PWM + turn_pwm;
            Right_Motor_Out = Balance_PWM - turn_pwm;

            if (Is_Reach_B_Point()) {
                b_reached = 1U;

                Current_Task = TASK_IDLE;
                Left_Motor_Out = 0.0f;
                Right_Motor_Out = 0.0f;
            }
        } else {
            Clear_Dynamic_Loops();

            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    case TASK_4_SPIN_BALANCE:
        /*
         * 任务四：原地旋转平衡。
         */
        Target_Speed_L = 0.0f;
        Target_Speed_R = 0.0f;

        if (Is_Angle_Safe()) {
            const float spin_value = 200.0f;

            angle_delta = Calculate_Velocity_PWM(0.0f,
                                                 Actual_Speed_L,
                                                 Actual_Speed_R);

            dynamic_target_angle = Mechanical_Middle + angle_delta;

            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle,
                                                 dynamic_target_angle);

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

        Left_Motor_Out = 0.0f;
        Right_Motor_Out = 0.0f;
        break;
    }

    if (Debug_Mode_Enable == 1U) {
        test_is_B_point = b_reached;
    }

    Motor_Output_Limit(&Left_Motor_Out, &Right_Motor_Out);

    Motor_Set_Speed((int16_t)Left_Motor_Out,
                    (int16_t)Right_Motor_Out);
}
