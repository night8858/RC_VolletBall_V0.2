#include "ball_track.h"
#include "top_ctrl.h"
#include "pid.h"
#include "math.h"

extern Ball_Pos RX_ball_pos;

ball_track_target_t ball_track_target;

pid_type_def track_pid;

//追踪pid的初始化
void ball_track_pid_init(void)
{
    
    fp32 PID_track[3];
    PID_track[0] = BALL_TRACK_PID_KP;
    PID_track[1] = BALL_TRACK_PID_KI;
    PID_track[2] = BALL_TRACK_PID_KD;
    PID_init(&track_pid , PID_POSITION , PID_track , BALL_TRACK_PID_MAX_OUTPUT , BALL_TRACK_PID_MAX_IOUT);

}

//计算球的跟踪目标值
void ball_track_calc(void)
{

    ball_track_target.offset_pos = hypot((RX_ball_pos.ball_pos_x - BALL_TRACK_TARGET_X ) , (RX_ball_pos.ball_pos_y - BALL_TRACK_TARGET_Y ));

    ball_track_target.angle   =   atan2(RX_ball_pos.ball_pos_x - BALL_TRACK_TARGET_X , RX_ball_pos.ball_pos_y - BALL_TRACK_TARGET_Y );
    ball_track_target.speed  = PID_calc(&track_pid , ball_track_target.offset_pos , 0);

    if(RX_ball_pos.ball_pos_z < BALL_TRACK_TARGET_DEEPTH)
    {ball_track_target.hit_falg = 1;}
    else
    {ball_track_target.hit_falg = 0;}

}