#include "main.h"
#include "top_ctrl.h"
#include "math.h"
#include "pid.h"
#include "can.h"
#include "can_recv.h"
#include "chassis.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

extern DM4340_motor_data_t DM4340_Date[3]; // 引用DM4340回传数据结构体
extern motor_control_t motor_control;
extern DBUSDecoding_Type DBUS_ReceiveData; // 底盘发来的dbus的数据

double T[3];     // T是引入万能公式的一个变量。令tan（THETA/2） = T,这样就可以将式子中相关与THETA的复杂运算给简化成用T表示。
double THETA[3]; // THETA变量表示三个电机的角度，可由万能公式的T解出

s_pid_absolute_t DM_motor_pid_p[4] = {0};
s_pid_absolute_t DM_motor_pid_s[4] = {0};
Top_Pos delta_position;

double A[3] = {0};
double B[3] = {0};
double C[3] = {0};

double K[3] = {0};
double U[3] = {0};
double V[3] = {0};

void top_contorl_Task(void const *argument)
{
    DM_Motor_Init();
    DM_Motor_pid_Init();
    motor_init(&motor_control);
    while (1)
    {
       juggle_Mode();
       Institution_Pos_Contorl();
       osDelay(2);
    }
}

void DM_Motor_Init(void)
{

    for (int i = 0; i < 2; i++)
    {
        start_motor(&hcan2, 0x01);
        osDelay(1000);
        start_motor(&hcan2, 0x02);
        osDelay(1000);
        start_motor(&hcan2, 0x03);
        osDelay(1000);
    }
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

void DMdate_feedback_update(void)
{
}
//颠球模式
void juggle_Mode(void)
{
    DM4340_Date[0].target_angle = float_constrain(353 - (DBUS_ReceiveData.ch1) / 6, 323, 353);
    DM4340_Date[1].target_angle = float_constrain(293 - (DBUS_ReceiveData.ch1) / 6, 262, 293);
    DM4340_Date[2].target_angle = float_constrain(303 - (DBUS_ReceiveData.ch1) / 6, 271, 303);
    DM_Motor_pid_Calc();

    		MD_motor_SendCurrent-(&hcan2,0x01,0,0,0,0,DM4340_Date[0].out_current);
			//osDelay(2);
			MD_motor_SendCurrent(&hcan2,0x02,0,0,0,0,DM4340_Date[1].out_current);
			//osDelay(2);
			MD_motor_SendCurrent(&hcan2,0x03,0,0,0,0,DM4340_Date[2].out_current);
			osDelay(2);
}

// 球拍姿态解算
void delta_arm_solution(void)
{

    A[0] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) - 2 * delta_position.x * (R1 - R2)) / (2 * L1);
    B[0] = -(R1 - R2 - delta_position.x);
    C[0] = delta_position.z;

    A[1] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (delta_position.x - sqrt(3) * delta_position.y) * (R1 - R2)) / L1;
    B[1] = -2 * (R1 - R2) - (delta_position.x - sqrt(3) * delta_position.y);
    C[1] = 2 * delta_position.z;

    A[2] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (delta_position.x - sqrt(3) * delta_position.y) * (R1 - R2)) / L1;
    B[2] = -2 * (R1 - R2) - (delta_position.x + sqrt(3) * delta_position.y);
    C[2] = 2 * delta_position.z;

    K[0] = A[0] + B[0];
    U[0] = 2 * C[0];
    V[0] = A[0] - B[0];

    K[1] = A[1] + B[1];
    U[1] = 2 * C[1];
    V[1] = A[1] - B[1];

    K[2] = A[2] + B[2];
    U[2] = 2 * C[2];
    V[2] = A[2] - B[2];

    T[0] = (-U[0] - sqrt(U[0] * U[0] - 4 * K[0] * V[0])) / (2 * K[0]);
    T[1] = (-U[1] - sqrt(U[1] * U[1] - 4 * K[1] * V[1])) / (2 * K[1]);
    T[2] = (-U[2] - sqrt(U[2] * U[2] - 4 * K[2] * V[2])) / (2 * K[2]);

    THETA[0] = (180 * (2 * atan(T[1]))) / PI;
    THETA[1] = (180 * (2 * atan(T[2]))) / PI;
    THETA[2] = (180 * (2 * atan(T[0]))) / PI;
}
// DM电机的pid参数初始化
void DM_Motor_pid_Init(void)
{

    pid_abs_param_init(&DM_motor_pid_p[0], 1.1, 0, 7, 1000, 1000);
    pid_abs_param_init(&DM_motor_pid_p[1], 1.1, 0, 7, 1000, 1000);
    pid_abs_param_init(&DM_motor_pid_p[2], 1.1, 0, 7, 1000, 1000);

    pid_abs_param_init(&DM_motor_pid_s[0], 1.8, 0, 0, 1000, 10);
    pid_abs_param_init(&DM_motor_pid_s[1], 1.8, 0, 0, 1000, 10);
    pid_abs_param_init(&DM_motor_pid_s[2], 1.8, 0, 0, 1000, 10);
}
// DM电机的pid计算
void DM_Motor_pid_Calc(void)
{

    DM_motor_pid_p[0].NowError = DM4340_Date[0].target_angle - DM4340_Date[0].serial_angle;
    DM_motor_pid_p[1].NowError = DM4340_Date[1].target_angle - DM4340_Date[1].real_angle;
    DM_motor_pid_p[2].NowError = DM4340_Date[2].target_angle - DM4340_Date[2].real_angle;

    PID_AbsoluteMode(&DM_motor_pid_p[0]);
    
    PID_AbsoluteMode(&DM_motor_pid_p[1]);
    PID_AbsoluteMode(&DM_motor_pid_p[2]);

    DM4340_Date[0].target_speed = DM_motor_pid_p[0].PIDout;
    DM4340_Date[1].target_speed = DM_motor_pid_p[1].PIDout;
    DM4340_Date[2].target_speed = DM_motor_pid_p[2].PIDout;

    DM_motor_pid_s[0].NowError = DM4340_Date[0].target_speed - DM4340_Date[0].esc_back_speed;
    DM_motor_pid_s[1].NowError = DM4340_Date[1].target_speed - DM4340_Date[1].esc_back_speed;
    DM_motor_pid_s[2].NowError = DM4340_Date[2].target_speed - DM4340_Date[2].esc_back_speed;

    PID_AbsoluteMode(&DM_motor_pid_s[0]);
    PID_AbsoluteMode(&DM_motor_pid_s[1]);
    PID_AbsoluteMode(&DM_motor_pid_s[2]);

    DM4340_Date[0].out_current = DM_motor_pid_s[0].PIDout;
    DM4340_Date[1].out_current = DM_motor_pid_s[1].PIDout;
    DM4340_Date[2].out_current = DM_motor_pid_s[2].PIDout;
}

