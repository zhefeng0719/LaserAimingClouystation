/** 
* @brief FOC���Ŀ������ļ�
* @version 1.0
* @file FOC.c
* @author ������С��
* @date 2025-06-25
* @note ���ļ�ʵ����FOC�������ĳ�ʼ�����ǶȻ����ٶȻ����������Լ�����������õȹ��ܡ�
Ӳ����MT6701�ű����� Ӳ��IIC
     STM32F407VET6
���һ:
	2804��ˢ��� �Ż� Ť�أ�0.06N/m ����12V
	���Ʒ�ʽ�� �ǶȻ� + �ٶȻ� ����PID �޵�����
�����:
	4015��ˢ��� �Ÿ�Ƭ Ť��:0.36N/m ����24V
	���Ʒ�ʽ�� ���ǶȻ� ���ٶȻ� �޵�����
��Ҫ��ʼ��FOCinit()���������ز�ͬ���޸�PID����������Ĭ�ϵ����
������Ҫ���������ȴ���Χ�豸ȫ���ϵ磬IIC������ɵ���FOCinit()����
* @note ���ļ�������MT6701.h��stm32f4xx_hal.h���ļ���
*/
#include "FOC.h"
#include "math.h"
#include "stdio.h"
#include "MT6701.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define FOC1MODE 1//0:�ǶȻ�+�ٶȻ� 1:���ǶȻ�

pid angle1,angle2,speed1;
Torque Brush1,Brush2;

unsigned int FOCOK;

float zero_electric_angle_1,zero_electric_angle_2;// 0���Ƕ�
float full_rotations_1,full_rotations_2;    				//��ǰ��תȦ��
float angle_prev_1,angle_prev_2;		 					// �ϴνǶȣ�����λ�û���
float vel_angle_prev_1,vel_angle_pv_2;					// �ϴνǶȣ������ٶȻ���
float last_angle_1 = 0.0f;							// ���1�ϴνǶȣ������ٶȻ���

/**
* @brief ��ʼ��FOC������
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
* @brief ���1�ٶȻ�������
* @param target_speed Ŀ���ٶ�
* @param angle ��ǰ�Ƕ�
* @retval �ٶȻ�PID���
*/
float Speed_Control_1(float target_speed,float angle)
{
    float angle_now = angle*180.0f/3.1415926f;
    static float filtered_speed = 0.0f;
    float alpha = 0.95f; // ��ͨ�˲�ϵ������Χ0~1��ԽСԽƽ��
    float raw_speed = angle_now - last_angle_1;
    // һ�׵�ͨ�˲�
    filtered_speed = alpha * raw_speed + (1.0f - alpha) * filtered_speed;
    speed1.Set = target_speed;
    speed1.Actual = filtered_speed;
    last_angle_1 = angle_now;
    speed1.err = speed1.Set - speed1.Actual;
    speed1.integral += speed1.err;
    // �����޷�
    if(speed1.integral > speed1.integral_limit) speed1.integral = speed1.integral_limit;
    if(speed1.integral < -speed1.integral_limit) speed1.integral = -speed1.integral_limit;
    // PID����
    speed1.voltage = speed1.Kp * speed1.err
                   + speed1.Ki * speed1.integral
                   + speed1.Kd * (speed1.err - speed1.err_last);
    speed1.err_last = speed1.err;
    return speed1.voltage; // �����ٶȻ�PID���
}

/**
* @brief ���1�ǶȻ�������
* @param Angle_Err �Ƕ����
* @retval �ǶȻ�PID���
*/
float Angle_Control_1(float Angle_Err)
{
    static float filtered_err = 0.0f;
    float alpha = 0.95f; // ��ͨ�˲�ϵ������Χ0~1��ԽСԽƽ��
    float PWM_Out;
    // һ�׵�ͨ�˲�
    filtered_err = alpha * Angle_Err + (1.0f - alpha) * filtered_err;
    angle1.err = filtered_err;
    angle1.integral += angle1.err;
	if(fabs(filtered_err) < 0.01f){
        filtered_err = 0.0f;
	}
    PWM_Out = angle1.Kp * angle1.err + angle1.Ki * angle1.integral + angle1.Kd * (angle1.err - angle1.err_last);
    angle1.integral = angle1.integral > angle1.integral_limit ? angle1.integral_limit : (angle1.integral < (-angle1.integral_limit) ? (-angle1.integral_limit) : angle1.integral); // �޷�
    angle1.err_last = angle1.err;
    return PWM_Out;
}

/**
* @brief ���1�Ƕ��趨
* @param Angle �����Ƕ�
* @retval None
*/
void Set_Angle_1(float Angle)
{
	if(FOC1MODE==0)
	{
	//�ǶȻ�ȡ
	angle1.Actual = get_Angle_1(); // ��ȡʵ�ʽǶ�
	angle1.Set = Angle; // ����Ŀ��Ƕ�
	//�Ƕ�������
	angle1.err = (angle1.Set - DIR*angle1.Actual)*180.0f/PI; // ����Ŀ��Ƕ���ʵ�ʽǶȵ����
	angle1.output = Angle_Control_1(angle1.err);
	setTorque_1(Speed_Control_1(angle1.output,angle1.Actual),_electricalAngle_1());	
	}
	else if(FOC1MODE==1)
	{
		//�ǶȻ�ȡ
		angle1.Actual = get_Angle_1(); // ��ȡʵ�ʽǶ�
		angle1.Set = Angle; // ����Ŀ��Ƕ�
		//�Ƕ�������
		angle1.err = (angle1.Set - DIR*angle1.Actual)*180.0f/PI; // ����Ŀ��Ƕ���ʵ�ʽǶȵ����
		angle1.output = Angle_Control_1(angle1.err);
		setTorque_1(angle1.output,_electricalAngle_1());
	}
}

/**
* @brief ���1�ٶ��趨
* @param speed �����ٶ�
* @retval None
*/
void Set_Speed_1(float speed)
{
	angle1.Actual = get_Angle_1(); // ��ȡʵ�ʽǶ�
	//�ٶ��趨
	speed1.Set = speed;
	float torque_cmd = Speed_Control_1(speed1.Set,angle1.Actual);
	setTorque_1(torque_cmd, _electricalAngle_1());
}

/**
* @brief ���2�ǶȻ�������
* @param Angle_Err �Ƕ����
* @retval �ǶȻ�PID���
*/
float Angle_Control_2(float Angle_Err)
{
	float PWM_Out;
	angle2.err=Angle_Err;
	angle2.integral+=angle2.err;
	PWM_Out=angle2.Kp * angle2.err + angle2.Ki * angle2.integral + angle2.Kd * (angle2.err-angle2.err_last);
	angle2.integral=angle2.integral>2000?2000:(angle2.integral<(-2000)?(-2000):angle2.integral);//�޷�
	angle2.err_last=angle2.err;
	return PWM_Out;
}

/**
* @brief ���2�Ƕ��趨
* @param Angle �����Ƕ�
* @retval None
*/
void Set_Angle_2(float Angle)
{
	//�ǶȻ�ȡ
	angle2.Actual = get_Angle_2(); // ��ȡʵ�ʽǶ�
	angle2.Set = Angle; // ����Ŀ��Ƕ�
	//�Ƕ�������
	angle2.err = (angle2.Set - DIR*angle2.Actual)*180.0f/PI; // ����Ŀ��Ƕ���ʵ�ʽǶȵ����
	angle2.output = Angle_Control_2(angle2.err);
	setTorque_2(angle2.output,_electricalAngle_2());
}


/**
* @brief ��ȡ���1ʵ�ʽǶȣ���Ȧ����
* @retval �����ƽǶ�
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
* @brief ��ȡ���2ʵ�ʽǶȣ���Ȧ����
* @retval �����ƽǶ�
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
* @brief �Ƕȹ�һ����0~2��
* @param angle ����Ƕ�
* @retval ��һ����Ƕ�
*/
float _normalizeAngle(float angle)
{
	float a = fmod(angle,2*PI);
	return a >= 0 ? a : (a+ 2*PI);
}

/**
* @brief ��ȡ���1��Ƕȣ���Ƕ�=��е�Ƕȡ�������
* @retval ���1��Ƕ�
*/
float _electricalAngle_1(void)
{
	return _normalizeAngle((float)(DIR * PP) * MT6701read()-zero_electric_angle_1);
}

/**
* @brief ��ȡ���2��Ƕȣ���Ƕ�=��е�Ƕȡ�������
* @retval ���2��Ƕ�
*/
float _electricalAngle_2(void)
{
	return _normalizeAngle((float)(DIR * PP2) * MT6701read2()-zero_electric_angle_2);
}

/**
* @brief ���1��������
* @param Uq ���ص�ѹ
* @param angle_el ��Ƕ�
* @retval None
*/
void setTorque_1(float Uq,float angle_el)
{
	//Լ�����ص�ѹ
	Uq = _constrain(Uq,-Brush1.voltage_power_supply/2,Brush1.voltage_power_supply/2);
	angle_el = _normalizeAngle(angle_el);
	//Park��任
	Brush1.Ualpha = -Uq*sin(angle_el);
	Brush1.Ubeta  = Uq*cos(angle_el); 
	//Clark��任
	Brush1.Ua = Brush1.Ualpha + Brush1.voltage_power_supply/2;
	Brush1.Ub = (sqrt(3)*Brush1.Ubeta-Brush1.Ualpha)/2 + Brush1.voltage_power_supply/2;
	Brush1.Uc = (-Brush1.Ualpha-sqrt(3)*Brush1.Ubeta)/2 + Brush1.voltage_power_supply/2;
	//Լ����ѹ
	Brush1.Ua = _constrain(Brush1.Ua,0.0f,Brush1.voltage_limit);
	Brush1.Ub = _constrain(Brush1.Ub,0.0f,Brush1.voltage_limit);
	Brush1.Uc = _constrain(Brush1.Uc,0.0f,Brush1.voltage_limit);
	//ռ�ձȼ���
	Brush1.dc_a = _constrain(Brush1.Ua/Brush1.voltage_power_supply,0.0f,1.0f);
	Brush1.dc_b = _constrain(Brush1.Ub/Brush1.voltage_power_supply,0.0f,1.0f);
	Brush1.dc_c = _constrain(Brush1.Uc/Brush1.voltage_power_supply,0.0f,1.0f);
	//PWMռ�ձ�����
	TIM3->CCR1 =(Brush1.dc_a*2800);
	TIM3->CCR2 =(Brush1.dc_b*2800);
	TIM3->CCR3 =(Brush1.dc_c*2800);
}

/**
* @brief ���2��������
* @param Uq ���ص�ѹ
* @param angle_el ��Ƕ�
* @retval None
*/
void setTorque_2(float Uq,float angle_el)
{
	//Լ�����ص�ѹ
	Uq = _constrain(Uq,-Brush2.voltage_power_supply/2,Brush2.voltage_power_supply/2);
	angle_el = _normalizeAngle(angle_el);
	//Park��任
	Brush2.Ualpha = -Uq*sin(angle_el);
	Brush2.Ubeta  = Uq*cos(angle_el); 
	//Clark��任
	Brush2.Ua = Brush2.Ualpha + Brush2.voltage_power_supply/2;
	Brush2.Ub = (sqrt(3)*Brush2.Ubeta-Brush2.Ualpha)/2 + Brush2.voltage_power_supply/2;
	Brush2.Uc = (-Brush2.Ualpha-sqrt(3)*Brush2.Ubeta)/2 + Brush2.voltage_power_supply/2;
	//Լ����ѹ
	Brush2.Ua = _constrain(Brush2.Ua,0.0f,Brush2.voltage_limit);
	Brush2.Ub = _constrain(Brush2.Ub,0.0f,Brush2.voltage_limit);
	Brush2.Uc = _constrain(Brush2.Uc,0.0f,Brush2.voltage_limit);
	//ռ�ձȼ���
	Brush2.dc_a = _constrain(Brush2.Ua/Brush2.voltage_power_supply,0.0f,1.0f);
	Brush2.dc_b = _constrain(Brush2.Ub/Brush2.voltage_power_supply,0.0f,1.0f);
	Brush2.dc_c = _constrain(Brush2.Uc/Brush2.voltage_power_supply,0.0f,1.0f);
	//PWMռ�ձ�����
	TIM4->CCR1 =(Brush2.dc_a*2800);
	TIM4->CCR2 =(Brush2.dc_b*2800);
	TIM4->CCR3 =(Brush2.dc_c*2800);
}

