#include "app_control.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_sensor.h"
#include "bsp_sa100.h"
#include "tim.h"

/* ========================================================================= */
/* 鎺у埗鍛ㄦ湡涓庣紪鐮佸櫒閰嶇疆                                                       */
/* ========================================================================= */
#define CTRL_PERIOD_MS              5U
#define CTRL_PERIOD_S               0.005f
#define CTRL_PERIOD_MS_F            5.0f
#define ENCODER_PPR                 1560.0f
#define RPM_SCALE                   (60000.0f / (CTRL_PERIOD_MS_F * ENCODER_PPR))

#define ENC_L_DIR                   1
#define ENC_R_DIR                  -1

/* 鍊捐瀹夊叏鍖恒€侻echanical_Middle=145.30 鏃讹紝瀹夊叏鍖哄ぇ绾︿负涓績涓婁笅 20 搴� */
#define ANGLE_SAFE_MIN              215.0f
#define ANGLE_SAFE_MAX              254.0f
#define ANGLE_RESCUE_MIN            218.0f
#define ANGLE_RESCUE_MAX            256.0f

/* ========================================================================= */
/* 闄愬箙鍙傛暟                                                                   */
/* ========================================================================= */
#define VELOCITY_INTEGRAL_LIMIT     40.0f
#define VELOCITY_ANGLE_LIMIT        1.5f
#define POSITION_PULSE_LIMIT        4500.0f
#define POSITION_ANGLE_LIMIT        3.0f
#define BALANCE_PWM_LIMIT           850.0f
#define TRACK_PWM_LIMIT             420.0f
#define TASK3_FORWARD_RPM_MAX       180.0f
#define TASK3_FORWARD_RPM_MIN       0.0f

/* 閫熷害鐜笉瑕� 5ms 鐩存帴鏇存柊锛氱紪鐮佸櫒閲忓寲澶矖锛�20ms 鏇存柊鏇寸ǔ */
#define VELOCITY_LOOP_DIV           4U
#define VELOCITY_LOOP_PERIOD_S      ((float)VELOCITY_LOOP_DIV * CTRL_PERIOD_S)
#define VELOCITY_LPF_ALPHA          0.30f
#define VELOCITY_DEADBAND_RPM       3.0f

/* 鐘舵€佹満鏃堕棿 */
#define STABILIZE_TIME_MS           800U
#define TASK2_HOLD_TIME_MS          12000U

/* B 鐐规娴� */
#define B_POINT_DEBOUNCE_COUNT      4U
#define B_POINT_MIN_TIME_MS         1000U
#define B_POINT_MIN_TRAVEL_PULSE    700.0f
#ifndef B_POINT_BLACK_MIN_COUNT
#define B_POINT_BLACK_MIN_COUNT     7U
#endif
#ifndef B_POINT_CENTER_BLACK_MIN_COUNT
#define B_POINT_CENTER_BLACK_MIN_COUNT 4U
#endif

/* ========================================================================= */
/* 鍏ㄥ眬鎺у埗鍙橀噺涓庡弬鏁�                                                         */
/* ========================================================================= */
volatile uint8_t Debug_Mode_Enable = 0U;
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

/*
 * 璋冨弬椤哄簭寤鸿锛�
 * 1. 鍏堝彧寮€ Balance_Kp / Balance_Kd锛岃鎵嬫壎鐩寸珛鍙互绔欎綇銆�
 * 2. 鍐嶆墦寮€閫熷害鐜€傞€熷害鐜緭鍑虹殑鏄洰鏍囪搴﹀亸缃紝涓嶆槸 PWM銆�
 * 3. 鏈€鍚庡啀鎵撳紑浣嶇疆鐜€佸惊杩圭幆銆�
 */
volatile float Balance_Kp = 123.0f;
volatile float Balance_Kd = 8.0f;
volatile float Mechanical_Middle = 235.799f;

/* 閫熷害鐜粯璁ゅ€煎凡缁忓ぇ骞呴檷浣庛€傚師鏉ョ殑 0.18 浼氭妸缂栫爜鍣� 1 涓剦鍐叉斁澶ф垚鏁扮櫨 PWM 鍐插嚮銆� */
volatile float Velocity_Kp = 0.006f;
volatile float Velocity_Ki = 0.0f;
volatile float Velocity_Direction = 1.0f;       /* 濡傛灉涓€寮€閫熷害鐜洿瀹规槗鍊掞紝鎶婂畠鏀规垚 -1.0f */

/* 浣嶇疆鐜粯璁ゅ厛鍏抽棴銆傞€熷害鐜皟绋冲悗锛屽啀鎱㈡參寮€鍚€� */
volatile float Position_Kp = 0.0f;
volatile float Position_Kd = 0.0f;

volatile float Track_Kp = 18.0f;
volatile float Track_Kd = 8.0f;
volatile float Track_Pure_Base_PWM = 850.0f;
volatile float Track_Balance_Target_RPM = 130.0f;
volatile float Track_Base_Speed = 650.0f;
volatile float Track_Turn_Direction = 1.0f;
volatile float Balance_Start_Duty = 0.0f;
volatile float Track_Ramp_RPM_Per_S = 70.0f;
volatile float Angle_Filter_Alpha = 0.65f;
volatile float Deadzone_PWM = 0.0f;

volatile Target_Task_e Current_Task = TASK_IDLE;
volatile Control_Phase_e Control_Phase = CTRL_PHASE_WAIT_UPRIGHT;
volatile uint32_t Task_Timer_Ms = 0U;

float Actual_Speed_L = 0.0f;
float Actual_Speed_R = 0.0f;
float Target_Speed_L = 0.0f;
float Target_Speed_R = 0.0f;
float Balance_PWM = 0.0f;
float Velocity_PWM = 0.0f;          /* 瀹為檯鍚箟锛氶€熷害鐜緭鍑虹殑鐩爣瑙掑害鍋忕疆锛屽崟浣嶈繎浼间负搴� */
float Track_PWM = 0.0f;
float Left_Motor_Out = 0.0f;
float Right_Motor_Out = 0.0f;
float Dynamic_Target_Angle = 235.7f;
float Position_Sum_Pulse = 0.0f;
float Current_Track_Error = 0.0f;
float Forward_Target_RPM = 0.0f;

static float velocity_integral = 0.0f;
static uint8_t velocity_loop_cnt = 0U;
static float velocity_speed_acc = 0.0f;
static float velocity_speed_lpf = 0.0f;
static float velocity_angle_hold = 0.0f;

static float balance_angle_f = 235.70f;
static float balance_last_angle_f = 235.7f;
static float last_position_error = 0.0f;
static float last_track_error = 0.0f;
static float track_error_f = 0.0f;
static float travel_pulse_abs = 0.0f;
static uint8_t b_point_debounce = 0U;
static uint32_t phase_timer_ms = 0U;
static Target_Task_e last_task = TASK_IDLE;

/* ========================================================================= */
/* 宸ュ叿鍑芥暟                                                                   */
/* ========================================================================= */
static float Limit_Float(float v, float min_v, float max_v)
{
    if (v > max_v) return max_v;
    if (v < min_v) return min_v;
    return v;
}

static float Abs_Float(float v)
{
    return (v >= 0.0f) ? v : -v;
}

static uint8_t Is_Angle_Safe(void)
{
    return (Pendulum_Angle > ANGLE_SAFE_MIN && Pendulum_Angle < ANGLE_SAFE_MAX) ? 1U : 0U;
}

static uint8_t Is_Angle_Rescuable(void)
{
    return (Pendulum_Angle > ANGLE_RESCUE_MIN && Pendulum_Angle < ANGLE_RESCUE_MAX) ? 1U : 0U;
}

static int16_t Read_Encoder_And_Clear(TIM_HandleTypeDef *htim)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    return count;
}

static void Clear_Dynamic_Loops(void)
{
    velocity_integral = 0.0f;
    velocity_loop_cnt = 0U;
    velocity_speed_acc = 0.0f;
    velocity_speed_lpf = 0.0f;
    velocity_angle_hold = 0.0f;

    balance_angle_f = Pendulum_Angle;
    balance_last_angle_f = balance_angle_f;
    last_position_error = 0.0f;
    last_track_error = 0.0f;
    track_error_f = 0.0f;
    Forward_Target_RPM = 0.0f;
    Sensor_Reset_Track_State();
}

static float Apply_Deadzone(float pwm)
{
    if (pwm > 0.0f && pwm < Deadzone_PWM) return Deadzone_PWM;
    if (pwm < 0.0f && pwm > -Deadzone_PWM) return -Deadzone_PWM;
    return pwm;
}

void Motor_Output_Limit(float *left, float *right)
{
    *left = Limit_Float(*left, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
    *right = Limit_Float(*right, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
}

uint8_t Is_Reach_B_Point(void)
{
    uint8_t all_black = Sensor_Count_Black();
    uint8_t center_black = Sensor_Count_Center_Black();
    uint8_t detected = 0U;

    if (Task_Timer_Ms < B_POINT_MIN_TIME_MS || travel_pulse_abs < B_POINT_MIN_TRAVEL_PULSE) {
        b_point_debounce = 0U;
        return 0U;
    }

    if (all_black >= B_POINT_BLACK_MIN_COUNT) detected = 1U;
    if (center_black >= B_POINT_CENTER_BLACK_MIN_COUNT) detected = 1U;

    if (detected) {
        if (b_point_debounce < B_POINT_DEBOUNCE_COUNT) b_point_debounce++;
    } else {
        b_point_debounce = 0U;
    }

    return (b_point_debounce >= B_POINT_DEBOUNCE_COUNT) ? 1U : 0U;
}

/* ========================================================================= */
/* 鎺у埗鐜�                                                                     */
/* ========================================================================= */
float Calculate_Track_PWM(void)
{
    float raw_error = Sensor_Get_Track_Error();
    float delta_error;
    float output;

    track_error_f += Angle_Filter_Alpha * (raw_error - track_error_f);
    Current_Track_Error = track_error_f;

    delta_error = Current_Track_Error - last_track_error;
    last_track_error = Current_Track_Error;

    output = Current_Track_Error * Track_Kp + delta_error * Track_Kd;
    output = Limit_Float(output, -TRACK_PWM_LIMIT, TRACK_PWM_LIMIT);
    test_track_err = Current_Track_Error;
    return output;
}

float Calculate_Balance_PWM(float current_angle, float target_angle)
{
    float error;
    float d_angle;
    float output;

    balance_angle_f += Angle_Filter_Alpha * (current_angle - balance_angle_f);

    error = balance_angle_f - target_angle;
    d_angle = balance_angle_f - balance_last_angle_f;
    balance_last_angle_f = balance_angle_f;

    output = error * Balance_Kp + d_angle * Balance_Kd;
    return Limit_Float(output, -BALANCE_PWM_LIMIT, BALANCE_PWM_LIMIT);
}

/*
 * 閫熷害鐜細20ms 鏇存柊涓€娆★紝杈撳嚭鐩爣瑙掑害淇閲忋€�
 * 娉ㄦ剰锛氳繖閲岀粷瀵逛笉瑕佺洿鎺ヨ緭鍑� PWM銆傞€熷害鐜彧鍏佽杞诲井绉诲姩骞宠　鐐广€�
 */
float Calculate_Velocity_Angle(float target_speed, float actual_l, float actual_r)
{
    float current_speed_5ms;
    float current_speed_20ms;
    float error;
    float output;

    current_speed_5ms = (actual_l + actual_r) * 0.5f;
    velocity_speed_acc += current_speed_5ms;
    velocity_loop_cnt++;

    if (velocity_loop_cnt < VELOCITY_LOOP_DIV) {
        return velocity_angle_hold;
    }

    current_speed_20ms = velocity_speed_acc / (float)velocity_loop_cnt;
    velocity_speed_acc = 0.0f;
    velocity_loop_cnt = 0U;

    velocity_speed_lpf += VELOCITY_LPF_ALPHA * (current_speed_20ms - velocity_speed_lpf);

    error = target_speed - velocity_speed_lpf;

    if (error > -VELOCITY_DEADBAND_RPM && error < VELOCITY_DEADBAND_RPM) {
        error = 0.0f;
    }

    if (Is_Angle_Safe()) {
        velocity_integral += error * VELOCITY_LOOP_PERIOD_S;
        velocity_integral = Limit_Float(velocity_integral,
                                        -VELOCITY_INTEGRAL_LIMIT,
                                        VELOCITY_INTEGRAL_LIMIT);
    } else {
        velocity_integral = 0.0f;
    }

    output = Velocity_Kp * error + Velocity_Ki * velocity_integral;
    output *= Velocity_Direction;
    output = Limit_Float(output, -VELOCITY_ANGLE_LIMIT, VELOCITY_ANGLE_LIMIT);

    velocity_angle_hold = output;
    return velocity_angle_hold;
}

static float Calculate_Position_Angle(float delta_pulse_avg)
{
    float position_error;
    float position_delta;
    float output;

    Position_Sum_Pulse += delta_pulse_avg;
    Position_Sum_Pulse = Limit_Float(Position_Sum_Pulse, -POSITION_PULSE_LIMIT, POSITION_PULSE_LIMIT);

    position_error = 0.0f - Position_Sum_Pulse;
    position_delta = position_error - last_position_error;
    last_position_error = position_error;

    output = position_error * Position_Kp + position_delta * Position_Kd;
    return Limit_Float(output, -POSITION_ANGLE_LIMIT, POSITION_ANGLE_LIMIT);
}

/* ========================================================================= */
/* 鐘舵€佹満涓庡垵濮嬪寲                                                             */
/* ========================================================================= */
static void Enter_New_Task(Target_Task_e task)
{
    (void)task;
    Task_Timer_Ms = 0U;
    phase_timer_ms = 0U;
    Control_Phase = CTRL_PHASE_WAIT_UPRIGHT;
    Balance_PWM = 0.0f;
    Velocity_PWM = 0.0f;
    Track_PWM = 0.0f;
    Left_Motor_Out = 0.0f;
    Right_Motor_Out = 0.0f;
    Target_Speed_L = 0.0f;
    Target_Speed_R = 0.0f;
    Dynamic_Target_Angle = Mechanical_Middle;
    Position_Sum_Pulse = 0.0f;
    travel_pulse_abs = 0.0f;
    b_point_debounce = 0U;
    test_is_B_point = 0U;
    Clear_Dynamic_Loops();
}

void Reset_Control_Variables(void)
{
    Task_Timer_Ms = 0U;
    phase_timer_ms = 0U;
    Control_Phase = CTRL_PHASE_WAIT_UPRIGHT;
    Balance_PWM = 0.0f;
    Velocity_PWM = 0.0f;
    Track_PWM = 0.0f;
    Left_Motor_Out = 0.0f;
    Right_Motor_Out = 0.0f;
    Target_Speed_L = 0.0f;
    Target_Speed_R = 0.0f;
    Dynamic_Target_Angle = Mechanical_Middle;
    Position_Sum_Pulse = 0.0f;
    Current_Track_Error = 0.0f;
    Forward_Target_RPM = 0.0f;
    travel_pulse_abs = 0.0f;
    b_point_debounce = 0U;
    test_is_B_point = 0U;
    Clear_Dynamic_Loops();
}

void Control_Task_Init(void)
{
    Sensor_Init();
    Reset_Control_Variables();
    last_task = Current_Task;
}

static void Update_Debug(int16_t enc_l, int16_t enc_r)
{
    if (Debug_Mode_Enable == 1U) {
        test_screen_task = (uint8_t)Current_Task;
        test_angle = Pendulum_Angle;
        test_enc_L = enc_l;
        test_enc_R = enc_r;
        test_track_raw_bits = Sensor_Get_Raw_Bits();
    }
}

/* ========================================================================= */
/* 5ms 涓绘帶鍒跺惊鐜�                                                             */
/* ========================================================================= */
void Control_Task_Loop_5ms(void)
{
    int16_t enc_l;
    int16_t enc_r;
    float delta_pulse_avg;
    float position_angle;
    float turn_pwm;
    uint8_t b_reached = 0U;

    if (Current_Task != last_task) {
        Enter_New_Task(Current_Task);
        last_task = Current_Task;
    }

    if (Motor_Test_Mode_Enable == 1U) {
        test_screen_task = (uint8_t)Current_Task;
        test_angle = Pendulum_Angle;
        test_enc_L = (int16_t)(ENC_L_DIR * Read_Encoder_And_Clear(&htim3));
        test_enc_R = (int16_t)(ENC_R_DIR * Read_Encoder_And_Clear(&htim4));
        Actual_Speed_L = (float)test_enc_L * RPM_SCALE;
        Actual_Speed_R = (float)test_enc_R * RPM_SCALE;
        test_track_raw_bits = Sensor_Get_Raw_Bits();
        test_pwm_L = Limit_Float(test_pwm_L, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
        test_pwm_R = Limit_Float(test_pwm_R, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
        Motor_Set_Speed((int16_t)test_pwm_L, (int16_t)test_pwm_R);
        return;
    }

    enc_l = (int16_t)(ENC_L_DIR * Read_Encoder_And_Clear(&htim3));
    enc_r = (int16_t)(ENC_R_DIR * Read_Encoder_And_Clear(&htim4));
    Actual_Speed_L = (float)enc_l * RPM_SCALE;
    Actual_Speed_R = (float)enc_r * RPM_SCALE;
    delta_pulse_avg = ((float)enc_l + (float)enc_r) * 0.5f;
    travel_pulse_abs += (Abs_Float((float)enc_l) + Abs_Float((float)enc_r)) * 0.5f;
    Update_Debug(enc_l, enc_r);

    if (Current_Task == TASK_IDLE) {
        Motor_Set_Speed(0, 0);
        Reset_Control_Variables();
        return;
    }

    Task_Timer_Ms += CTRL_PERIOD_MS;
    phase_timer_ms += CTRL_PERIOD_MS;

    switch (Current_Task) {

    case TASK_1_PURE_TRACK:
        Velocity_PWM = Track_Pure_Base_PWM;
        Track_Base_Speed = Track_Pure_Base_PWM;
        Track_PWM = Calculate_Track_PWM();
        turn_pwm = Track_Turn_Direction * Track_PWM;

        Left_Motor_Out = Velocity_PWM + turn_pwm;
        Right_Motor_Out = Velocity_PWM - turn_pwm;

        if (Is_Reach_B_Point()) {
            b_reached = 1U;
            Current_Task = TASK_IDLE;
            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    case TASK_2_STAY_BALANCE:
        Target_Speed_L = 0.0f;
        Target_Speed_R = 0.0f;

        if (Is_Angle_Rescuable()) {
            if (Control_Phase == CTRL_PHASE_WAIT_UPRIGHT && Is_Angle_Safe()) {
                Control_Phase = CTRL_PHASE_STABILIZE;
                phase_timer_ms = 0U;
                Clear_Dynamic_Loops();
            }

            Velocity_PWM = Calculate_Velocity_Angle(0.0f, Actual_Speed_L, Actual_Speed_R);
            position_angle = Calculate_Position_Angle(delta_pulse_avg);
            Dynamic_Target_Angle = Mechanical_Middle + Velocity_PWM + position_angle;
            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle, Dynamic_Target_Angle);

            Left_Motor_Out = Apply_Deadzone(Balance_PWM);
            Right_Motor_Out = Apply_Deadzone(Balance_PWM);
        } else {
            Clear_Dynamic_Loops();
            Left_Motor_Out = 0.0f;
            Right_Motor_Out = 0.0f;
        }
        break;

    case TASK_3_TRACK_BALANCE:
        if (Is_Angle_Rescuable()) {
            if (Control_Phase == CTRL_PHASE_WAIT_UPRIGHT && Is_Angle_Safe()) {
                Control_Phase = CTRL_PHASE_STABILIZE;
                phase_timer_ms = 0U;
                Clear_Dynamic_Loops();
            }

            if (Control_Phase == CTRL_PHASE_STABILIZE && phase_timer_ms >= STABILIZE_TIME_MS) {
                Control_Phase = CTRL_PHASE_RUN;
                phase_timer_ms = 0U;
            }

            if (Control_Phase == CTRL_PHASE_RUN) {
                Forward_Target_RPM += Track_Ramp_RPM_Per_S * CTRL_PERIOD_S;
                Forward_Target_RPM = Limit_Float(Forward_Target_RPM, 0.0f, Track_Balance_Target_RPM);
                Forward_Target_RPM = Limit_Float(Forward_Target_RPM, 0.0f, TASK3_FORWARD_RPM_MAX);
            } else {
                Forward_Target_RPM = 0.0f;
            }

            Target_Speed_L = Forward_Target_RPM;
            Target_Speed_R = Forward_Target_RPM;

            Velocity_PWM = Calculate_Velocity_Angle(Forward_Target_RPM, Actual_Speed_L, Actual_Speed_R);
            Dynamic_Target_Angle = Mechanical_Middle + Velocity_PWM;
            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle, Dynamic_Target_Angle);
            Track_PWM = Calculate_Track_PWM();
            turn_pwm = Track_Turn_Direction * Track_PWM;

            Left_Motor_Out = Apply_Deadzone(Balance_PWM + turn_pwm);
            Right_Motor_Out = Apply_Deadzone(Balance_PWM - turn_pwm);

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
        Target_Speed_L = 0.0f;
        Target_Speed_R = 0.0f;
        if (Is_Angle_Safe()) {
            Velocity_PWM = Calculate_Velocity_Angle(0.0f, Actual_Speed_L, Actual_Speed_R);
            Dynamic_Target_Angle = Mechanical_Middle + Velocity_PWM;
            Balance_PWM = Calculate_Balance_PWM(Pendulum_Angle, Dynamic_Target_Angle);

            Left_Motor_Out = Apply_Deadzone(Balance_PWM + 180.0f);
            Right_Motor_Out = Apply_Deadzone(Balance_PWM - 180.0f);
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

    test_is_B_point = b_reached;
    Motor_Output_Limit(&Left_Motor_Out, &Right_Motor_Out);
    Motor_Set_Speed((int16_t)Left_Motor_Out, (int16_t)Right_Motor_Out);
}
