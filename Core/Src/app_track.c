#include "app_track.h"
#include "bsp_track.h"
#include "bsp_motor.h"

// 传感器极性配置：压到黑线输出1则设为1，输出0则设为0
#define SENSOR_ON_LINE  1  

// 电机最大 PWM 限幅 (请根据你定时器的实际 ARR 重装载值修改)
#define MAX_SPEED 1000 

// 实例化全局控制对象 (针对橡胶轮，P和D参数建议从较小的值开始试)
Track_Ctrl_t TrackCtrl = {
    .error = 0.0f,
    .last_error = 0.0f,
    .base_speed = 300, // 默认巡航速度 (换橡胶轮后建议先用中低速测试)
    .kp = 30.0f,       // 转向比例 P (橡胶轮抓地力强，P值要比麦轮小)	 
    .kd = 10.0f        // 转向微分 D (先调好P能转弯了，再加D消除抖动) 
};

void App_Track_Init(void) {
    BSP_Track_Init();
}

// 核心计算：获取当前偏离中心线的误差值
float App_Track_Get_Error(void) {
    Track_Data_t t;
    BSP_Track_Read(&t);

    // 统一逻辑：让 t.chX 为 1 时代表“压到了线”
#if SENSOR_ON_LINE == 0
    t.ch1 = !t.ch1; t.ch2 = !t.ch2; t.ch3 = !t.ch3; t.ch4 = !t.ch4;
    t.ch5 = !t.ch5; t.ch6 = !t.ch6; t.ch7 = !t.ch7; t.ch8 = !t.ch8;
#endif

    int sum = t.ch1 + t.ch2 + t.ch3 + t.ch4 + t.ch5 + t.ch6 + t.ch7 + t.ch8;

    // --- 脱线保护机制 ---
    if (sum == 0) {
        // 冲出赛道时，根据最后一次的误差记忆，强制打死方向盘拉回
        if (TrackCtrl.last_error > 0) return 5.0f;  // 狠狠往右找
        if (TrackCtrl.last_error < 0) return -5.0f; // 狠狠往左找
        return 0.0f;
    }

    // --- 加权平均法计算偏差 ---
    // 探头分布： 1   2   3   4 || 5   6   7   8
    // 权重分配：-4  -3  -2  -1 || 1   2   3   4
    float weighted_sum = t.ch1 * (-4.0f) + t.ch2 * (-3.0f) +
        t.ch3 * (-2.0f) + t.ch4 * (-1.0f) +
        t.ch5 * (1.0f) + t.ch6 * (2.0f) +
        t.ch7 * (3.0f) + t.ch8 * (4.0f);

    // 计算当前误差，但不在这里立刻覆盖 last_error，留给 D 控制器用
    TrackCtrl.error = weighted_sum / (float)sum;

    return TrackCtrl.error;
}

// 执行循迹动作：自动计算偏差并控制差速底盘走线
void App_Track_Follow_Line(int base_speed) {
    float current_error = App_Track_Get_Error();

    // --- PD 控制器计算转向调节量 ---
    float delta_error = current_error - TrackCtrl.last_error;
    float turn_adjust = (current_error * TrackCtrl.kp) + (delta_error * TrackCtrl.kd);

    // 计算完 D 项后，将当前误差存入 last_error，供下一次周期使用
    TrackCtrl.last_error = current_error;

    // --- 差速底盘运动学计算 ---
    // 如果发现小车往反方向跑，请把这里的加减号对调：
    // left_speed = base_speed - (int)turn_adjust; 
    // right_speed = base_speed + (int)turn_adjust;
    int left_speed = base_speed + (int)turn_adjust;
    int right_speed = base_speed - (int)turn_adjust;

    // 软件限幅保护 (防止 PWM 超出定时器量程，并允许急转弯时反转)
    if (left_speed > MAX_SPEED)   left_speed = MAX_SPEED;
    if (left_speed < -MAX_SPEED)  left_speed = -MAX_SPEED;
    if (right_speed > MAX_SPEED)  right_speed = MAX_SPEED;
    if (right_speed < -MAX_SPEED) right_speed = -MAX_SPEED;

    // --- 控制电机输出 ---
    // 【需修改】：替换为你自己的直流电机底层驱动函数
    // 例如：LeftMotor_Set(left_speed); RightMotor_Set(right_speed);
    Motor_Set_Speed(left_speed, right_speed);
}
