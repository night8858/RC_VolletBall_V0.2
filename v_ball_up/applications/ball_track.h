#ifndef __BALL_TRACK_H__
#define __BALL_TRACK_H__
#include "struct_typedef.h"

#define BALL_TRACK_TARGET_X 320        // 目标点的x坐标
#define BALL_TRACK_TARGET_Y 240        // 目标点的y坐标
#define BALL_TRACK_TARGET_DEEPTH 0.32f // 目标点的深度  //36???怪   //0.38 OOK
#define BALL_TRACK_TARGET_DOWN 0.06f    // 绿地0.04
#define BALL_TRACK_TARGET_DEEPTH_MAX 0.4f
#define BALL_TRACK_TARGET_FORCE 30
#define BALL_TRACK_TARGET_FORCE_BEGIN 30
#define BALL_TRACK_TARGET_FORCE_MAX 30
#define RACKET_TO_CAMERA_DISTANCE 0 // 球拍到摄像头的距离
#define RACQUET_CENTER_HIGHT 0.08f  // 球拍中心高度

#define RACKET_UP_HIGHT 0.10f // 球拍上升高度

//speed PID
#define BALL_TRACK_PID_KP 8.0f // PID参数KP
#define BALL_TRACK_PID_KI 0.0f  // PID参数KI
#define BALL_TRACK_PID_KD 0.0f  // PID参数KD

//目前的PID  1.8  0  1.0
#define BALL_TRACK_PID_MAX_OUTPUT 15000
#define BALL_TRACK_PID_MAX_IOUT 1000

//POS PID
//p:5.6 i:0 d:-0.9
#define POS_PID_KP 5.0f   // PID参数KP  // 绿地5.6
#define POS_PID_KI 0.0f   // PID参数KI
#define POS_PID_KD 0.4f  // PID参数KD
 
 //19  0  2.8
#define POS_PID_MAX_OUTPUT 10000
#define POS_PID_MAX_IOUT 1000

typedef struct 
{ 
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter


typedef struct
{
    float motor_real_speed; // 电机实际速度

    float speed;              // 方向速度
    float angle;              // 角度
    float distance;           // 距离
    float last_distance;      // 上次距离
    float change_of_distance; // 距离变化;

    float ball_speed; // 球速约值

    float real_distance; // 球距离最低点的实际距离
    float real_distance_K; // 球距离最低点的实际距离

    float offset_pos_x;      // 偏移x位置
    float last_offset_pos_x; // 上次偏移x位置
    float offset_speed_x;    // 偏移x速度

    float offset_pos_y;      // 偏移y位置
    float last_offset_pos_y; // 上次偏移y位置
    float offset_speed_y;    // 偏移y速度

    float offset_pos;      // 偏移位置大小
    float last_offset_pos; // 上次偏移位置大小
    float offset_speed;    // 偏移速度

    float offset_angle; // 偏移角度
    float WhiteRatio;   // 白球比例

    int hit_flag; // 击打标志位   0:未击打  1:击打  2:大力击球
    int mode_falg;
    int frist_hit_flag;

    int hit_num; // 击打次数

} ball_track_target_t;


typedef struct
{
    
    //float max_relative_angle; //rad
    //float min_relative_angle; //rad

    float relative_pos;     //rad
    float relative_pos_set; //rad

    float motor_gyro;         //rad/s
    float motor_gyro_set;
    float motor_speed;
    float raw_cmd_current;
    float current_set;
    int16_t given_current;

} pos_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;
    float lsat_err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;

} POS_PID_t;


void ball_track_pid_init(void);
void ball_track_calc(void);
float kalmanFilter(KFP *kfp, float input);
static fp32 POS_PID_calc(POS_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static void Pos_PID_init(POS_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
#endif /* __BALL_TRACK_H__ */
