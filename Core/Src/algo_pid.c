#include "algo_pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float int_limit, float out_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = int_limit;
    pid->output_limit = out_limit;
    
    PID_Reset(pid); // 【已修复】：去掉了乱码，使用正确的重置函数名
}

float PID_Compute_Position(PID_t *pid, float target, float measured)
{
    pid->error = target - measured;
    pid->integral += pid->error;
    
    if (pid->integral > pid->integral_limit)   pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit)  pid->integral = -pid->integral_limit;
    
    pid->output = (pid->kp * pid->error) + \
                  (pid->ki * pid->integral) + \
                  (pid->kd * (pid->error - pid->last_error));
    
    pid->last_error = pid->error;
    
    if (pid->output > pid->output_limit)   pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit)  pid->output = -pid->output_limit;
    
    return pid->output;
}

float PID_Compute_Incremental(PID_t *pid, float target, float measured)
{
    pid->error = target - measured;
    
    float delta_output = (pid->kp * (pid->error - pid->last_error)) + \
                         (pid->ki * pid->error) + \
                         (pid->kd * (pid->error - 2.0f * pid->last_error + pid->prev_error));
    
    pid->output += delta_output;
    
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    
    if (pid->output > pid->output_limit)   pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit)  pid->output = -pid->output_limit;
    
    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}
