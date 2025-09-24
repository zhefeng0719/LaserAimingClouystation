/** 
* @brief FOC核心控制器文件
* @version 1.0
* @file FOC.c
* @author 花衬衫小伙
* @date 2025-06-25
* @note 本文件实现了FOC控制器的初始化、角度环和速度环控制器，以及电机力矩设置等功能。
硬件：MT6701磁编码器 硬件IIC
     STM32F407VET6
电机一:
	2804无刷电机 磁环 扭矩：0.06N/m 供电12V
	控制方式： 角度环 + 速度环 串级PID 无电流环
电机二:
	4015无刷电机 磁钢片 扭矩:0.36N/m 供电24V
	控制方式： 纯角度环 无速度环 无电流环
需要初始化FOCinit()函数，负载不同需修改PID参数，程序默认电机二
程序需要缓启动，等待外围设备全部上电，IIC就绪后可调用FOCinit()函数
* @note 该文件依赖于MT6701.h、stm32f4xx_hal.h等文件。
*/
#include "FOC.h"
#include "math.h"
#include "stdio.h"
#include "MT6701.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define FOC1MODE 1//0:角度环+速度环 1:纯角度环

pid angle1,angle2,speed1;
Torque Brush1,Brush2;

unsigned int FOCOK;

float zero_electric_angle_1,zero_electric_angle_2;// 0点电角度
float full_rotations_1,full_rotations_2;    				//当前旋转圈数
float angle_prev_1,angle_prev_2;		 					// 上次角度（用于位置环）
float vel_angle_prev_1,vel_angle_pv_2;					// 上次角度（用于速度环）
float last_angle_1 = 0.0f;							// 电机1上次角度（用于速度环）

/**
* @brief 初始化FOC控制器
* @retval None
*/
void FOCinit()
{
	const float benefit1= 2.0f,benefit2 = 10.0f;
	if(FOC1MODE==1)
	{
	angle1.Kp=0.35f*benefit1;//0.03
	angle1.Ki=0.008f*benefit1;//0.000
	angle1.Kd=2.0f*benefit1;//0.00
	angle1.integral_limit = 1000;
	}
	else if(FOC1MODE==0){
	angle1.Kp=0.00f*benefit1;
	angle1.Ki=0.000f*benefit1;
	angle1.Kd=0.00f*benefit1;
	angle1.integral_limit = 10000;
	}
	speed1.Kp=3.0f;//2.0
	speed1.Ki=0.15f;//0.005
	speed1.Kd=0.000f;//0.00
	speed1.integral_limit = 100000;

	angle2.Kp=0.030f*benefit2;//0.03
	angle2.Ki=0.0005f*benefit2;//0.0005                                                                                        
	angle2.Kd=0.06f*benefit2;//0.00
	
	Brush1.voltage_limit=12.3f;
	Brush1.voltage_power_supply=11.3f;
	Brush2.voltage_limit=24.3f;
	Brush2.voltage_power_supply=22.6f;
	HAL_Delay(1000);
	setTorque_1(3, _3PI_2);
	HAL_Delay(500);
	setTorque_2(3, _3PI_2);
	zero_electric_angle_1 = _electricalAngle_1();
	HAL_Delay(500);
	zero_electric_angle_2 = _electricalAngle_2();
	setTorque_1(0, _3PI_2);
	HAL_Delay(500);
	setTorque_2(0, _3PI_2);
	FOCOK=1;
}

/**
* @brief 电机1速度环控制器
* @param target_speed 目标速度
* @param angle 当前角度
* @retval 速度环PID输出
*/
float Speed_Control_1(float target_speed,float angle)
{
    float angle_now = angle*180.0f/3.1415926f;
    static float filtered_speed = 0.0f;
    float alpha = 0.95f; // 低通滤波系数，范围0~1，越小越平滑
    float raw_speed = angle_now - last_angle_1;
    // 一阶低通滤波
    filtered_speed = alpha * raw_speed + (1.0f - alpha) * filtered_speed;
    speed1.Set = target_speed;
    speed1.Actual = filtered_speed;
    last_angle_1 = angle_now;
    speed1.err = speed1.Set - speed1.Actual;
    speed1.integral += speed1.err;
    // 积分限幅
    if(speed1.integral > speed1.integral_limit) speed1.integral = speed1.integral_limit;
    if(speed1.integral < -speed1.integral_limit) speed1.integral = -speed1.integral_limit;
    // PID计算
    speed1.voltage = speed1.Kp * speed1.err
                   + speed1.Ki * speed1.integral
                   + speed1.Kd * (speed1.err - speed1.err_last);
    speed1.err_last = speed1.err;
    return speed1.voltage; // 返回速度环PID输出
}

/**
* @brief 电机1角度环控制器
* @param Angle_Err 角度误差
* @retval 角度环PID输出
*/
float Angle_Control_1(float Angle_Err)
{
    static float filtered_err = 0.0f;
    float alpha = 0.95f; // 低通滤波系数，范围0~1，越小越平滑
    float PWM_Out;
    // 一阶低通滤波
    filtered_err = alpha * Angle_Err + (1.0f - alpha) * filtered_err;
    angle1.err = filtered_err;
    angle1.integral += angle1.err;
	if(fabs(filtered_err) < 0.01f){
        filtered_err = 0.0f;
	}
    PWM_Out = angle1.Kp * angle1.err + angle1.Ki * angle1.integral + angle1.Kd * (angle1.err - angle1.err_last);
    angle1.integral = angle1.integral > angle1.integral_limit ? angle1.integral_limit : (angle1.integral < (-angle1.integral_limit) ? (-angle1.integral_limit) : angle1.integral); // 限幅
    angle1.err_last = angle1.err;
    return PWM_Out;
}

/**
* @brief 电机1角度设定
* @param Angle 期望角度
* @retval None
*/
void Set_Angle_1(float Angle)
{
	if(FOC1MODE==0)
	{
	//角度获取
	angle1.Actual = get_Angle_1(); // 获取实际角度
	angle1.Set = Angle; // 设置目标角度
	//角度误差计算
	angle1.err = (angle1.Set - DIR*angle1.Actual)*180.0f/PI; // 计算目标角度与实际角度的误差
	angle1.output = Angle_Control_1(angle1.err);
	setTorque_1(Speed_Control_1(angle1.output,angle1.Actual),_electricalAngle_1());	
	}
	else if(FOC1MODE==1)
	{
		//角度获取
		angle1.Actual = get_Angle_1(); // 获取实际角度
		angle1.Set = Angle; // 设置目标角度
		//角度误差计算
		angle1.err = (angle1.Set - DIR*angle1.Actual)*180.0f/PI; // 计算目标角度与实际角度的误差
		angle1.output = Angle_Control_1(angle1.err);
		setTorque_1(angle1.output,_electricalAngle_1());
	}
}

/**
* @brief 电机1速度设定
* @param speed 期望速度
* @retval None
*/
void Set_Speed_1(float speed)
{
	angle1.Actual = get_Angle_1(); // 获取实际角度
	//速度设定
	speed1.Set = speed;
	float torque_cmd = Speed_Control_1(speed1.Set,angle1.Actual);
	setTorque_1(torque_cmd, _electricalAngle_1());
}

/**
* @brief 电机2角度环控制器
* @param Angle_Err 角度误差
* @retval 角度环PID输出
*/
float Angle_Control_2(float Angle_Err)
{
	float PWM_Out;
	angle2.err=Angle_Err;
	angle2.integral+=angle2.err;
	PWM_Out=angle2.Kp * angle2.err + angle2.Ki * angle2.integral + angle2.Kd * (angle2.err-angle2.err_last);
	angle2.integral=angle2.integral>2000?2000:(angle2.integral<(-2000)?(-2000):angle2.integral);//限幅
	angle2.err_last=angle2.err;
	return PWM_Out;
}

/**
* @brief 电机2角度设定
* @param Angle 期望角度
* @retval None
*/
void Set_Angle_2(float Angle)
{
	//角度获取
	angle2.Actual = get_Angle_2(); // 获取实际角度
	angle2.Set = Angle; // 设置目标角度
	//角度误差计算
	angle2.err = (angle2.Set - DIR*angle2.Actual)*180.0f/PI; // 计算目标角度与实际角度的误差
	angle2.output = Angle_Control_2(angle2.err);
	setTorque_2(angle2.output,_electricalAngle_2());
}


/**
* @brief 获取电机1实际角度（带圈数）
* @retval 弧度制角度
*/
float get_Angle_1(void)
{
	float val_1 = MT6701read();
	float d_angle_1 = val_1 - angle_prev_1;
	
	if( fabs(d_angle_1) > (0.8f*6.28318530718f) )
		full_rotations_1 += (d_angle_1 > 0) ? -1 : 1;
	angle_prev_1 = val_1;
	return (float)full_rotations_1 * 6.28318530718f + angle_prev_1;
};

/**
* @brief 获取电机2实际角度（带圈数）
* @retval 弧度制角度
*/
float get_Angle_2(void)
{
	float val_2 = MT6701read2();
	float d_angle_2 = val_2 - angle_prev_2;
	if( fabs(d_angle_2) > (0.8f*6.28318530718f) )
		full_rotations_2 += (d_angle_2 > 0) ? -1 : 1;
	angle_prev_2 = val_2;
	return (float)full_rotations_2 * 6.28318530718f + angle_prev_2;
};

/**
* @brief 角度归一化到0~2π
* @param angle 输入角度
* @retval 归一化后角度
*/
float _normalizeAngle(float angle)
{
	float a = fmod(angle,2*PI);
	return a >= 0 ? a : (a+ 2*PI);
}

/**
* @brief 获取电机1电角度，电角度=机械角度×极对数
* @retval 电机1电角度
*/
float _electricalAngle_1(void)
{
	return _normalizeAngle((float)(DIR * PP) * MT6701read()-zero_electric_angle_1);
}

/**
* @brief 获取电机2电角度，电角度=机械角度×极对数
* @retval 电机2电角度
*/
float _electricalAngle_2(void)
{
	return _normalizeAngle((float)(DIR * PP2) * MT6701read2()-zero_electric_angle_2);
}

/**
* @brief 电机1力矩设置
* @param Uq 力矩电压
* @param angle_el 电角度
* @retval None
*/
void setTorque_1(float Uq,float angle_el)
{
	//约束力矩电压
	Uq = _constrain(Uq,-Brush1.voltage_power_supply/2,Brush1.voltage_power_supply/2);
	angle_el = _normalizeAngle(angle_el);
	//Park逆变换
	Brush1.Ualpha = -Uq*sin(angle_el);
	Brush1.Ubeta  = Uq*cos(angle_el); 
	//Clark逆变换
	Brush1.Ua = Brush1.Ualpha + Brush1.voltage_power_supply/2;
	Brush1.Ub = (sqrt(3)*Brush1.Ubeta-Brush1.Ualpha)/2 + Brush1.voltage_power_supply/2;
	Brush1.Uc = (-Brush1.Ualpha-sqrt(3)*Brush1.Ubeta)/2 + Brush1.voltage_power_supply/2;
	//约束电压
	Brush1.Ua = _constrain(Brush1.Ua,0.0f,Brush1.voltage_limit);
	Brush1.Ub = _constrain(Brush1.Ub,0.0f,Brush1.voltage_limit);
	Brush1.Uc = _constrain(Brush1.Uc,0.0f,Brush1.voltage_limit);
	//占空比计算
	Brush1.dc_a = _constrain(Brush1.Ua/Brush1.voltage_power_supply,0.0f,1.0f);
	Brush1.dc_b = _constrain(Brush1.Ub/Brush1.voltage_power_supply,0.0f,1.0f);
	Brush1.dc_c = _constrain(Brush1.Uc/Brush1.voltage_power_supply,0.0f,1.0f);
	//PWM占空比设置
	TIM3->CCR1 =(Brush1.dc_a*2800);
	TIM3->CCR2 =(Brush1.dc_b*2800);
	TIM3->CCR3 =(Brush1.dc_c*2800);
}

/**
* @brief 电机2力矩设置
* @param Uq 力矩电压
* @param angle_el 电角度
* @retval None
*/
void setTorque_2(float Uq,float angle_el)
{
	//约束力矩电压
	Uq = _constrain(Uq,-Brush2.voltage_power_supply/2,Brush2.voltage_power_supply/2);
	angle_el = _normalizeAngle(angle_el);
	//Park逆变换
	Brush2.Ualpha = -Uq*sin(angle_el);
	Brush2.Ubeta  = Uq*cos(angle_el); 
	//Clark逆变换
	Brush2.Ua = Brush2.Ualpha + Brush2.voltage_power_supply/2;
	Brush2.Ub = (sqrt(3)*Brush2.Ubeta-Brush2.Ualpha)/2 + Brush2.voltage_power_supply/2;
	Brush2.Uc = (-Brush2.Ualpha-sqrt(3)*Brush2.Ubeta)/2 + Brush2.voltage_power_supply/2;
	//约束电压
	Brush2.Ua = _constrain(Brush2.Ua,0.0f,Brush2.voltage_limit);
	Brush2.Ub = _constrain(Brush2.Ub,0.0f,Brush2.voltage_limit);
	Brush2.Uc = _constrain(Brush2.Uc,0.0f,Brush2.voltage_limit);
	//占空比计算
	Brush2.dc_a = _constrain(Brush2.Ua/Brush2.voltage_power_supply,0.0f,1.0f);
	Brush2.dc_b = _constrain(Brush2.Ub/Brush2.voltage_power_supply,0.0f,1.0f);
	Brush2.dc_c = _constrain(Brush2.Uc/Brush2.voltage_power_supply,0.0f,1.0f);
	//PWM占空比设置
	TIM4->CCR1 =(Brush2.dc_a*2800);
	TIM4->CCR2 =(Brush2.dc_b*2800);
	TIM4->CCR3 =(Brush2.dc_c*2800);
}

