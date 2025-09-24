#include "MT6701.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define DIR 1//��ˢ�����ƫ����
#define PP 7//������
#define PP2 11//������
#define _3PI_2 4.71238898038
typedef struct//����pid�ṹ��
{
	float Set;//����Ŀ��ֵ
	float Actual;//������ʵֵ
	float err;//����ƫ��ֵ
	float err_last;//������һ��ƫ��ֵ
	float Kp,Ki,Kd;//������������֣�΢��
	float voltage;//�����ѹֵ
	float integral;//�������ֵ
	float integral_limit;//��������޷�
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







