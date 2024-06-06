#ifndef CHASSIS_H
#define CHASSIS_H

#include "struct_typedef.h"
#include "can_recv.h"
#include "pid.h"
#include "remote_control.h"

#define PI 3.141592653824f

#define M3505_MOTOR_SPEED_PID_KP 1.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 4000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 14000.0f

#define M3508_MOTOR_POSION_PID_KP 12.0f
#define M3508_MOTOR_POSION_PID_KI 0.0f
#define M3508_MOTOR_POSION_PID_KD 7.0f
#define M3508_MOTOR_POSION_PID_MAX_OUT 30000.0f
#define M3508_MOTOR_POSION_PID_MAX_IOUT 20000.0f


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

    M3508_PID_t m3508_motor_relative_angle_pid;
    pid_type_def m3508_motor_gyro_pid;

    int motor_ecd;
    int self_ecd_temp;
    
    float motor_gyro;         //rad/s
    float motor_gyro_set;

    float esc_back_position;
    float esc_back_position_last;
    
    float serial_angle;           //总角度
    float relative_angle_target;

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

    motor_3508_t M3508_M5;
    
} motor_control_t;



void motor_init(motor_control_t *init);

static void M3508_Pos_PID_init(M3508_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static void M3508_PID_clear(M3508_PID_t *M3508_pid_clear);

void motor_control_loop(motor_control_t *control_loop);

void motor_feedback_update(motor_control_t *feedback_update);
void motor_feedback_update(motor_control_t *feedback_update);

void M3508_motor_Pos_control(motor_3508_t *chassis_motor);

void M3508_motor_Pos_init(motor_3508_t *chassis_motor);

void Institution_Pos_Contorl(void);

void M3508_M5_Pos_init(void);

#endif 
