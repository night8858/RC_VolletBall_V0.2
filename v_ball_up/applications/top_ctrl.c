#include "main.h"
#include "top_ctrl.h"
#include "math.h"
#include "pid.h"
#include "can.h"
#include "can_recv.h"
#include "chassis.h"
#include "usart.h"
#include "bsp_usart.h"
#include "ball_track.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

extern DM4340_motor_data_t DM4340_Date[3];    // 引用DM4340回传数据结构体
extern motor_control_t motor_control;         // 引用底盘电机控制结构体
extern DBUSDecoding_Type DBUS_ReceiveData;    // 底盘发来的dbus的数据
extern ball_track_target_t ball_track_target; // 球追踪目标点

double T[3];     // T是引入万能公式的一个变量。令tan（THETA/2） = T,这样就可以将式子中相关与THETA的复杂运算给简化成用T表示。
double THETA[3]; // THETA变量表示三个电机的角度，可由万能公式的T解出

Top_Pos delta_position;
Ball_Pos RX_ball_pos;

double A[3] = {0};
double B[3] = {0};
double C[3] = {0};

double K[3] = {0};
double U[3] = {0};
double V[3] = {0};

// 球拍控制主循环
void top_contorl_Task(void const *argument)
{
    DM_Motor_Init();
    motor_init(&motor_control);
    // M3508_M5_Pos_init();
    ball_track_pid_init();
    while (1)
    {
        // 此处为手动操作模式
        if (DBUS_ReceiveData.switch_left == 1 && DBUS_ReceiveData.switch_right == 1)
        {
            juggle_Mode();
            // Institution_Pos_Contorl();
            osDelay(2);
        }

        // 此处为自动模式
        if (DBUS_ReceiveData.switch_left == 3 && DBUS_ReceiveData.switch_right == 3)
        {
            // ball_track_target.speed = 0.1;
            // ball_track_target.angle = 0;
            // chassis_cmd_aotu(&hcan1);
            ball_track_calc();
            juggle_Mode();

            osDelay(2);
        }
        // 复位
        if (DBUS_ReceiveData.switch_left != DBUS_ReceiveData.switch_right)
        {
            DM_Motor_Init();
            motor_init(&motor_control);
            ball_track_pid_init();
            osDelay(2);
        }
    }
}

// 达妙电机初始化
void DM_Motor_Init(void)
{

    for (int i = 0; i < 3; i++)
    {
        start_motor(&hcan2, 0x01);
        osDelay(500);
        start_motor(&hcan2, 0x02);
        osDelay(500);
        start_motor(&hcan2, 0x03);
        osDelay(500);
    }
    DM4340_Date[0].target_angle = (-(float_constrain(81, 81, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(145, 145, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(136, 136, 167)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
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

// 颠球模式
static void juggle_Mode(void)
{

    DM4340_Date[0].target_angle = (-(float_constrain(83 + (DBUS_ReceiveData.ch1) / 5, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + (DBUS_ReceiveData.ch1) / 5, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(137 + (DBUS_ReceiveData.ch1) / 5, 137, 167)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[0].target_angle / PI * 180, DM4340_Date[0].out_current);
}

// 自动颠球模式（暂时未启用）
static void juggle_Mode_auto(void)
{
    uint16_t angle = 0;

    if (ball_track_target.hit_falg == 1)
    {
        angle = 660;
    }
    DM4340_Date[0].target_angle = (-(float_constrain(81 + (angle) / 11, 81, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(145 + (angle) / 11, 145, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(135 + (angle) / 11, 135, 167)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
}

float RUD_DirAngle_c(float Angle)
{
    while (Angle > 18000 || Angle < 0)
    {
        if (Angle < 0)
        {
            Angle += 360;
        }
        if (Angle > 360)
        {
            Angle -= 360;
        }
    }
    return (float)Angle;
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
