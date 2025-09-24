#include "pid.h"
extern coordinate aim;
cloudpid Cloud1,Cloud2;
int target_lost_flag = 0; // 1表示丢失，0表示未丢失
void cloudpidinit()
{
    Cloud1.kp = 0.00065f;
    Cloud1.ki = 0.000023;
    Cloud1.kd = 0.0030;
    Cloud1.err_limit=160.0f;
    Cloud1.plus_limit = 60000.0f;
    Cloud1.en = 1;

    Cloud2.kp = 0.00032f;
    Cloud2.ki = 0.000012f;
    Cloud2.kd = 0.0020f;
    Cloud2.target = 0.0f;
    Cloud2.err_limit=240.0f;
    Cloud2.plus_limit = 60000.0f;
    Cloud2.en = 1;
    
}
void cloud1()
{
    Cloud1.target = 320.0f;
    Cloud1.err = Cloud1.target - aim.x;
    // 静态变量用于1秒内目标丢失检测
    static uint16_t lost_cnt = 0;
    if (Cloud1.err == 0.0f)
    {
        lost_cnt++;
        if (lost_cnt >= 200) // 1秒内都未检测到目标 
        {
            target_lost_flag = 1;
            lost_cnt = 200; // 防止溢出
        }
        return;
    } 
    else {
        lost_cnt = 0;
        target_lost_flag = 0;
    }
    // 误差限幅
    if (Cloud1.err > Cloud1.err_limit) Cloud1.err = Cloud1.err_limit;
    if (Cloud1.err < -Cloud1.err_limit) Cloud1.err = -Cloud1.err_limit;
    // 积分累加并限幅
    static float integral = 0.0f;
    integral += Cloud1.err;
    if (integral > Cloud1.plus_limit) integral = Cloud1.plus_limit;
    if (integral < -Cloud1.plus_limit) integral = -Cloud1.plus_limit;

    // 微分项一阶低通滤波，结合目标丢失标志位
    static float last_derivative = 0.0f;
    float raw_derivative = Cloud1.err - Cloud1.err_last;
    float derivative;
    const float alpha = 0.1f; // 滤波系数，0~1，越小越平滑
    if (target_lost_flag)
    {
        derivative = 0.0f;
        last_derivative = 0.0f;
    }
    else
    {
        derivative = last_derivative + alpha * (raw_derivative - last_derivative);
        last_derivative = derivative;
    }

    Cloud1.out = Cloud1.kp * Cloud1.err
               + Cloud1.ki * integral
               + Cloud1.kd * derivative;

    Cloud1.err_last = Cloud1.err;
}
void cloud2()
{
    Cloud2.target = 240.0f;
    Cloud2.err = Cloud2.target - aim.y;
    if(Cloud2.err==0.0f) return; // 如果误差为0，直接返回
    // 误差限幅
    if(Cloud2.err > Cloud2.err_limit) Cloud2.err = Cloud2.err_limit;
    if(Cloud2.err < -Cloud2.err_limit) Cloud2.err = -Cloud2.err_limit;

    // 积分累加并限幅
    static float integral = 0.0f;
    integral += Cloud2.err;
    if(integral > Cloud2.plus_limit) integral = Cloud2.plus_limit;
    if(integral < -Cloud2.plus_limit) integral = -Cloud2.plus_limit;

    // 微分项一阶低通滤波
    static float last_derivative = 0.0f;
    float raw_derivative = Cloud2.err - Cloud2.err_last;
    const float alpha = 0.15f; // 滤波系数，0~1，越小越平滑
    float derivative = last_derivative + alpha * (raw_derivative - last_derivative);
    last_derivative = derivative;

    Cloud2.out = Cloud2.kp * Cloud2.err
               + Cloud2.ki * integral
               + Cloud2.kd * derivative;

    Cloud2.err_last = Cloud2.err;
    // 此处可用 Cloud2.out 控制云台
}