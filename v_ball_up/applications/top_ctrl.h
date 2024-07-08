#ifndef _TOP_CTRL__
#define _TOP_CTRL_

#include "can_recv.h"
#include "can.h"
#include "chassis.h"

#define PI 3.141592653824f //圆周率

#define R1 89.0236f
#define L1 150.f         //连杆1，mm
#define La 150.f         //连杆2
#define R2 90.0f         //连接点到球拍中心的距离

#define DM_MOTOR_KP     68.0f    //位置增益  67
#define DM_MOTOR_KD     3.2f   //速度增益    可以尝试绑定发球速
#define DM_MOTOR_t_ff  -3.0f//前馈力矩


//过高5.6

//从上位机接收的球位置
typedef struct
{
	float ball_pos_x;//画面x

	float ball_pos_y;//画面y

	float ball_pos_z;//画面z（深度）

	float White_ratio;        //白色占比
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

void delay(int count);

//static void POS_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
//static fp32 POS_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

static void juggle_Mode(void);

static void test_Mode(void);

static void resolving_Mode(void);

static void hit_big_once(void);

static void hit_once(void);

static void hit_start(void);

static void juggle_Mode_auto(void);

static void back_to_zero(void);

#endif
