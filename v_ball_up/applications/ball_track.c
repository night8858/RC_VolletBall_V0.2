#include "ball_track.h"
#include "top_ctrl.h"
#include "pid.h"
#include "math.h"
#include "main.h"
#include "struct_typedef.h"

#include "user_lib.h"
extern Ball_Pos RX_ball_pos;
extern first_order_filter_type_t filter_angle;

ball_track_target_t ball_track_target;
pid_type_def track_pid;
M6020_PID_t pos_pid;
KFP KFP_height = {
    0.02, 0, 0, 0, 0.024, 0.543};

// 追踪pid的初始化
void ball_track_pid_init(void)
{
    fp32 PID_track[3];
    PID_track[0] = BALL_TRACK_PID_KP;
    PID_track[1] = BALL_TRACK_PID_KI;
    PID_track[2] = BALL_TRACK_PID_KD;
    // M6020_PID_init(&pos_pid, POS_PID_MAX_OUTPUT, POS_PID_MAX_IOUT, POS_PID_KP, POS_PID_KI, POS_PID_KD);
    PID_init(&track_pid, PID_POSITION, PID_track, BALL_TRACK_PID_MAX_OUTPUT, BALL_TRACK_PID_MAX_IOUT);
}

// static void offset_pos_pid_control(void)
//{
//     // 角度环，速度环串级pid
//     float motor_gyro_set = POS_PID_calc(&pos_pid, pos->relative_angle, pos->relative_angle_set, pos->motor_gyro);
//     pos->current_set = PID_calc(&pos->pos_gyro_pid, pos->motor_gyro, motor_gyro_set); // 控制值赋值
//     pos->given_current = (int16_t)(pos->current_set);
// }

// 计算球的跟踪目标值
void ball_track_calc(void)
{

    // 球的last偏移量
    ball_track_target.last_offset_pos_x = ball_track_target.offset_pos_x;
    ball_track_target.last_offset_pos_y = ball_track_target.offset_pos_y;
    // 计算球的偏移量
    ball_track_target.offset_pos_x = RX_ball_pos.ball_pos_x - BALL_TRACK_TARGET_X;
    ball_track_target.offset_pos_y = RX_ball_pos.ball_pos_y - BALL_TRACK_TARGET_Y;

    ball_track_target.last_distance = ball_track_target.real_distance;
    
    if (ball_track_target.real_distance > 3.5) { ball_track_target.real_distance = ball_track_target.last_distance; }
    ball_track_target.WhiteRatio = RX_ball_pos.White_ratio / 100.0f;

    switch (ball_track_target.hit_flag)
    {
    case 0:
        ball_track_target.real_distance = RX_ball_pos.ball_pos_z;
        break;
    case 1:
        ball_track_target.real_distance = RX_ball_pos.ball_pos_z + 0.10f;
        break;
    case 2:
        ball_track_target.real_distance = RX_ball_pos.ball_pos_z +  0.12f;
        break;

    default:
        break;
    }

    ball_track_target.change_of_distance = ball_track_target.last_distance - ball_track_target.real_distance;

    if (ball_track_target.WhiteRatio > 0.82f)
    {
        ball_track_target.hit_flag = 2;
    }
    else
    {
        ball_track_target.hit_flag = 0;
    }

    // 击球判断
    if (ball_track_target.real_distance < 0.30f)
    {
        if (ball_track_target.hit_flag == 2)
            return;
        if (ball_track_target.real_distance != 0)
            ball_track_target.hit_flag = 2;
    }
    else
    {
        ball_track_target.hit_flag = 0;
    }

    // 判断是否击球
    if (ball_track_target.real_distance < BALL_TRACK_TARGET_DEEPTH)
    {
        if (ball_track_target.hit_flag == 2)
            return;
        if (ball_track_target.real_distance != 0.0f)
        {
            ball_track_target.hit_flag = 1;
        }
        else
        {
            ball_track_target.hit_flag = 0;
        }
    }
    else
    {
        // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
        ball_track_target.hit_flag = 0;
    }

    // 设置死区大约是每个方向的2.5%
    if (RX_ball_pos.ball_pos_x < 16)
    {
        if (RX_ball_pos.ball_pos_x > -16)
        {
            RX_ball_pos.ball_pos_x = 0;
        }
    }
    if (RX_ball_pos.ball_pos_y < 12)
    {
        if (RX_ball_pos.ball_pos_y > -12)
        {
            RX_ball_pos.ball_pos_y = 0;
        }
    }

    ball_track_target.offset_pos = hypot((ball_track_target.offset_pos_x), (ball_track_target.offset_pos_y));
    // first_order_filter_cali( &filter_angle , (float)atan2(ball_offset_x, ball_offset_y));

    ball_track_target.angle = (float)atan2(ball_track_target.offset_pos_x, ball_track_target.offset_pos_y);
    // 运算pid
    ball_track_target.speed = -(PID_calc(&track_pid, ball_track_target.offset_pos, 0));
    ball_track_target.mode_falg = 1;
    switch (ball_track_target.hit_flag)
    {
    case 0:
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
        break;
    case 2:
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
        break;

    default:
        break;
    }
    // ball_track_target.real_distance_K = kalmanFilter(&KFP_height, ball_track_target.real_distance);
    //  ball_track_target.speed = 0;
    //  自动检测的部分
}

void hit_judge(void)
{
}

// 1. 结构体类型定义
// 2. 以高度为例 定义卡尔曼结构体并初始化参数

/** *卡尔曼滤波器 *@param KFP *kfp 卡尔曼结构体参数 * float input 需要滤波的参数的测量值（即传感器的采集值） *@return 滤波后的参数（最优值） */
float kalmanFilter(KFP *kfp, float input)
{

    // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->LastP + kfp->Q;
    // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
    // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}
