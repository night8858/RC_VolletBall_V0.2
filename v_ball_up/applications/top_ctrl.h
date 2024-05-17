#ifndef _TOP_CTRL__
#define _TOP_CTRL_

#include "can_recv.h"
#include "chassis.h"

#define PI 3.141592653824f //圆周率

#define R1 89.0236f
#define L1 150.f         //连杆1，mm
#define La 150.f       //连杆2
#define R2 90.0f         //连接点到球拍中心的距离


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
void DM_Motor_pid_Init(void);
void DM_Motor_pid_Calc(void);
void juggle_Mode(void);

#endif
