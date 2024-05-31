#ifndef _TOP_CTRL__
#define _TOP_CTRL_

#include "can_recv.h"
#include "chassis.h"

#define PI 3.141592653824f //圆周率

#define R1 89.0236f
#define L1 150.f         //连杆1，mm
#define La 150.f         //连杆2
#define R2 90.0f         //连接点到球拍中心的距离

#define DM_MOTOR_KP     67.0f    //位置增益
#define DM_MOTOR_KD     3.1f   //速度增益
#define DM_MOTOR_t_ff  -2.0f//前馈力矩

//从上位机接收的球位置
typedef struct
{
	float ball_pos_x;//画面x

	float ball_pos_y;//画面y

	float ball_pos_z;//画面z（深度）
}Ball_Pos;

//球拍的当前位置
typedef struct
{
	float x;
	float y;
	float z;
	
}Top_Pos;

void top_contorl_Task(void const * argument);
void start_motor(CAN_HandleTypeDef *Target_hcan, uint16_t id);
void DM_Motor_Init(void);
float float_constrain(float Value, float minValue, float maxValue);
void delta_arm_solution(void);

void juggle_Mode(void);

#endif
