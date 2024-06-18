#include "ball_track.h"
#include "top_ctrl.h"
#include "pid.h"
#include "math.h"
#include "main.h"

extern Ball_Pos RX_ball_pos;

ball_track_target_t ball_track_target;

pid_type_def track_pid;

// 追踪pid的初始化
void ball_track_pid_init(void)
{

    fp32 PID_track[3];
    PID_track[0] = BALL_TRACK_PID_KP;
    PID_track[1] = BALL_TRACK_PID_KI;
    PID_track[2] = BALL_TRACK_PID_KD;
    PID_init(&track_pid, PID_POSITION, PID_track, BALL_TRACK_PID_MAX_OUTPUT, BALL_TRACK_PID_MAX_IOUT);
}

// 计算球的跟踪目标值
void ball_track_calc(void)
{
    float ball_offset_x = RX_ball_pos.ball_pos_x - BALL_TRACK_TARGET_X;
    float ball_offset_y = RX_ball_pos.ball_pos_y - BALL_TRACK_TARGET_Y;

    ball_track_target.last_distance = ball_track_target.distance;
    ball_track_target.distance = RX_ball_pos.ball_pos_z;

    ball_track_target.change_of_distance = ball_track_target.last_distance - ball_track_target.distance ;
    // 设置死区
    if (fabs(ball_offset_x) < 20)
    {
        ball_offset_x = 0;
    }
    if (fabs(ball_offset_y) < 20)
    {
        ball_offset_y = 0;
    }

    ball_track_target.offset_pos = hypot((ball_offset_x), (ball_offset_y));
    ball_track_target.angle = atan2(ball_offset_x, ball_offset_y);

    // 运算pid
    ball_track_target.speed = -(PID_calc(&track_pid, ball_track_target.offset_pos, 0));

    // 自动检测的部分

    if (ball_track_target.distance < BALL_TRACK_TARGET_DEEPTH)
    {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
        if (ball_track_target.last_distance != 0)
        {
            ball_track_target.hit_falg = 1;
        }
        else
        {
            ball_track_target.hit_falg = 0;
        }
    }
    else
    {
        HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
        ball_track_target.hit_falg = 0;
    }

    
}
