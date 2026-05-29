#include "algo_chassis.h"

// 静态全局底盘参数
static Chassis_Param_t chassis;

/**
  * @brief  底盘物理参数初始化
  * @param  track: 左右轮轮距 (根据你小车实际测量填，如 150.0f mm)
  * @param  max_speed: 控制输出限幅 (这里填 8399.0f，对应最大 PWM)
  */
void Chassis_Init(float track, float max_speed)
{
    chassis.wheel_track = track;
    chassis.max_wheel_speed = max_speed;
}

/**
  * @brief  两驱底盘运动学逆解 (差速解算)
  * @param  linear_v:  期望纵向速度（正数前行，负数后退。通常由直立环输出与速度环输出叠加得到）
  * @param  angular_w: 期望转弯速度（正数右转，负数左转。由 8 路灰度寻迹 PID 算出）
  * @param  out_left_v:  [输出] 算出的左轮目标控制量 (直接传给左轮速度环PID或电机)
  * @param  out_right_v: [输出] 算出的右轮目标控制量 (直接传给右轮速度环PID或电机)
  */
void Chassis_Inverse_Kinematics(float linear_v, float angular_w, float *out_left_v, float *out_right_v)
{
    /* 两驱差速经典物理公式：
       左轮速度 = 线速度 - 角速度 * (轮距 / 2)
       右轮速度 = 线速度 + 角速度 * (轮距 / 2)
       
       提示：在实际电赛调车中，如果参数尚未标定到标准的物理单位（mm/s），
       通常可以直接简化将 (angular_w * chassis.wheel_track * 0.5f) 抽象定义为转向系数，
       即：左轮 = 线速度 - 转向修正量； 右轮 = 线速度 + 转向修正量。
    */
    
    float v_l = linear_v - (angular_w * chassis.wheel_track * 0.5f);
    float v_r = linear_v + (angular_w * chassis.wheel_track * 0.5f);
    
    // 针对两驱底盘进行双轮联动限幅保护
    if (v_l > chassis.max_wheel_speed)   v_l = chassis.max_wheel_speed;
    if (v_l < -chassis.max_wheel_speed)  v_l = -chassis.max_wheel_speed;
    
    if (v_r > chassis.max_wheel_speed)   v_r = chassis.max_wheel_speed;
    if (v_r < -chassis.max_wheel_speed)  v_r = -chassis.max_wheel_speed;
    
    // 将计算好的控制量传递出去
    *out_left_v = v_l;
    *out_right_v = v_r;
}
