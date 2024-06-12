#ifndef __BALL_TRACK_H__
#define __BALL_TRACK_H__

#define BALL_TRACK_TARGET_X 320            //目标点的x坐标
#define BALL_TRACK_TARGET_Y 240            //目标点的y坐标    
#define BALL_TRACK_TARGET_DEEPTH 0         //目标点的深度

#define RACKET_TO_CAMERA_DISTANCE 0        //球拍到摄像头的距离
#define RACQUET_CENTER_OFFSET_DISTANCE 0   //球拍中心偏移距离

#define BALL_TRACK_PID_KP 3        //PID参数KP
#define BALL_TRACK_PID_KI 0        //PID参数KI
#define BALL_TRACK_PID_KD 1.1        //PID参数KD

#define BALL_TRACK_PID_MAX_OUTPUT 3000
#define BALL_TRACK_PID_MAX_IOUT 1000

typedef struct
{
    float speed;             //方向速度
    float angle;             //角度

    float offset_pos;         //偏移位置
    float offset_angle;       //偏移角度

    int hit_falg;         //击打标志位

} ball_track_target_t;

void ball_track_pid_init(void);
void ball_track_calc(void);

#endif /* __BALL_TRACK_H__ */
