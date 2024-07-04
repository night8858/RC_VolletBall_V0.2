#ifndef CHASSIS_H
#define CHASSIS_H

#include "struct_typedef.h"
#include "can_recv.h"
#include "pid.h"
#include "remote_control.h"

#define PI 3.141592653824f

#define M3505_MOTOR_SPEED_PID_KP 12.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 3.4f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 3000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 14000.0f



#define M6020_MOTOR_SPEED_PID_KP 2.0f
#define M6020_MOTOR_SPEED_PID_KI 0.0f
#define M6020_MOTOR_SPEED_PID_KD 1.2f
#define M6020_MOTOR_SPEED_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 20000.0f


#define M6020_MOTOR_POSION_PID_KP 30000.0f
#define M6020_MOTOR_POSION_PID_KI 0.0f
#define M6020_MOTOR_POSION_PID_KD 7.0f
#define M6020_MOTOR_POSION_PID_MAX_OUT 30000.0f
#define M6020_MOTOR_POSION_PID_MAX_IOUT 20000.0f


#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192


typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;

} M6020_PID_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;

} M3508_PID_t;

typedef struct
{
    
    const motor_measure_t *gimbal_motor_measure;  //电机数据结构体
    M6020_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    uint16_t offset_ecd;
    float max_relative_angle; //rad
    float min_relative_angle; //rad

    float relative_angle;     //rad
    float relative_angle_set; //rad

    float motor_gyro;         //rad/s
    float motor_gyro_set;
    float motor_speed;
    float raw_cmd_current;
    float current_set;
    int16_t given_current;

} motor_6020_t;


typedef struct
{
    
    const motor_measure_t *chassis_motor_measure;  //电机数据结构体
    pid_type_def chassis_motor_gyro_pid;
    uint16_t offset_ecd;

    float motor_speed;
    float motor_speed_set;
    float raw_cmd_current;
    float current_set;
    int16_t given_current;

} motor_3508_t;


typedef struct
{
    const RC_ctrl_t *rc_ctrl;
 
    motor_6020_t M6020_M1;
    motor_6020_t M6020_M2;
    motor_6020_t M6020_M3;
    motor_6020_t M6020_M4;

    motor_3508_t M3508_M1;
    motor_3508_t M3508_M2;
    motor_3508_t M3508_M3;
    motor_3508_t M3508_M4;

} motor_control_t;


typedef struct
{
    float speed;
    float angle;
    float W;

} chassis_move_date;

typedef struct
{
    uint32_t rc_speed;
    uint32_t rc_angle;
    float speed;
    float angle;
    float W;

} chassis_auto_move_cmd;


//extern void chasis_Task(void const * argument);

static void motor_init(motor_control_t *init);

static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);

static void motor_control_loop(motor_control_t *control_loop);

static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor);

static void motor_feedback_update(motor_control_t *feedback_update);

static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear);

static void M3508_motor_speed_control(motor_3508_t *chassis_motor);

static void motor_feedback_update(motor_control_t *feedback_update);

static void movement_calc(void);

static void movement_calc_auto(void);

fp32 Find_min_Angle(int16_t angle1, fp32 angle2);
void AngleLoop_f(float *angle, float max);
#endif
