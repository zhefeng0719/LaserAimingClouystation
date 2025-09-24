#include "MT6701.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define DIR 1//无刷电机纠偏方向
#define PP 7//极对数
#define PP2 11//极对数
#define _3PI_2 4.71238898038
typedef struct//定义pid结构体
{
	float Set;//定义目标值
	float Actual;//定义真实值
	float err;//定义偏差值
	float err_last;//定义上一个偏差值
	float Kp,Ki,Kd;//定义比例，积分，微分
	float voltage;//定义电压值
	float integral;//定义积分值
	float integral_limit;//定义积分限幅
	float output;
}pid;
typedef struct
{
	float Ualpha;
	float Ubeta;
	float Ua;
	float Ub;
	float Uc;
	float dc_a;
	float dc_b;
	float dc_c;
	float voltage_limit;
	float voltage_power_supply;
}Torque;
void FOCinit();
float get_Angle_1(void);
void Set_Angle_1(float Angle);
void Set_Speed_1(float speed);
float Angle_Control_1(float Angle_Err);
float Speed_Control_1(float target_speed,float angle);
float _electricalAngle_1(void);
void setTorque_1(float Uq,float angle_el);

float get_Angle_2(void);
void Set_Angle_2(float Angle);
float Angle_Control_2(float Angle_Err);
float _electricalAngle_2(void);
void setTorque_2(float Uq,float angle_el);







