#ifndef __PID_H
#define __PID_H
#include "main.h"
//PID相关结构体
typedef struct
{
    float get;
    float target;
    float kp;
    float kd;
    float ki;
    float err;
    float err_last;
    float err_plus;
    float err_limit;
    float plus_limit;
    float out;
    int en;
} cloudpid;
void cloudpidinit();
void cloud1();
void cloud2();
#endif // __PID_H