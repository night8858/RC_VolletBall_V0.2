#include "main.h"
#include "pid.h"
#include "chassis.h"
#include "can_recv.h"
#include "user_lib.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "math.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"
/*对应电机数据,0~3 为1~4号动力电机3508,4~7为1~4号航向电机6020

   |                 ^ Y方向
   | 一号(6020)      |        二号(6020)
   |     (3508)      |           (3508)
   |                 |
   |   ——————————————|————————————————> X方向
   |                 |
   | 三号(6020)      |       四号（6020）
   |     (3508)      |          (3508)
   |                 |

*/

extern RC_ctrl_t rc_ctrl;
extern UART_HandleTypeDef huart1;
extern motor_measure_t motor_Date[8];

motor_control_t motor_control;
chassis_move_date chassis_float;
chassis_auto_move_cmd chassis_auto_move_cmd_data;

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define motor_total_pid_clear(motor_clear)                                         \
    {                                                                              \
        M6020_PID_clear(&(motor_clear)->M6020_M1.gimbal_motor_relative_angle_pid); \
        PID_clear(&(motor_clear)->M6020_M1.gimbal_motor_gyro_pid);                 \
        M6020_PID_clear(&(motor_clear)->M6020_M2.gimbal_motor_relative_angle_pid); \
        PID_clear(&(motor_clear)->M6020_M2.gimbal_motor_gyro_pid);                 \
        M6020_PID_clear(&(motor_clear)->M6020_M3.gimbal_motor_relative_angle_pid); \
        PID_clear(&(motor_clear)->M6020_M3.gimbal_motor_gyro_pid);                 \
        M6020_PID_clear(&(motor_clear)->M6020_M4.gimbal_motor_relative_angle_pid); \
        PID_clear(&(motor_clear)->M6020_M4.gimbal_motor_gyro_pid);                 \
                                                                                   \
        PID_clear(&(motor_clear)->M3508_M1.chassis_motor_gyro_pid);                \
        PID_clear(&(motor_clear)->M3508_M2.chassis_motor_gyro_pid);                \
        PID_clear(&(motor_clear)->M3508_M3.chassis_motor_gyro_pid);                \
        PID_clear(&(motor_clear)->M3508_M4.chassis_motor_gyro_pid);                \
    }

static void motor_init(motor_control_t *init);
static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor);
static void motor_feedback_update(motor_control_t *feedback_update);
static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear);
static void M3508_motor_speed_control(motor_3508_t *chassis_motor);
static void motor_control_loop(motor_control_t *control_loop);
static void motor_feedback_update(motor_control_t *feedback_update);
static void movement_calc(void);

// 主线程
void chasis_Task(void const *argument)
{
    // vTaskDelay(GIMBAL_TASK_INIT_TIME);
    motor_init(&motor_control);
    while (1)
    {
        //进入手动控制模式
        if (rc_ctrl.rc.s[0] == 1 && rc_ctrl.rc.s[1] == 1)
        {
            // 取得回传数据结构体
            motor_feedback_update(&motor_control);
            movement_calc();

            // PID控制计算和输出循环
            motor_control_loop(&motor_control);
            // vofa的测试代码
             //uart_dma_printf(&huart1,"%4.3f ,%4.3f\n",motor_control.M3508_M2.motor_speed , motor_control.M3508_M2.motor_speed_set);
            // uart_dma_printf(&huart1,"%4.3f ,%4.3f , %4.3f\n",motor_control.M3508_M1.motor_speed / 19, motor_control.M3508_M1.motor_speed_set / 19 , motor_control.M3508_M1.motor_speed - motor_control.M3508_M1.motor_speed_set);

            osDelay(2);
        }

         if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3)
         {
            motor_feedback_update(&motor_control);
            movement_calc_auto();
            // PID控制计算和输出循环
            motor_control_loop(&motor_control);
                // vofa调试用的代码
            osDelay(2);

         }
    }
}

// 电机数据的初始化
static void motor_init(motor_control_t *init)
{

    static const fp32 M6020_speed_pid[3] = {M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD};
    static const fp32 M3508_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    // static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    // 电机数据指针获取
    init->M6020_M1.gimbal_motor_measure = get_6020_M1_motor_measure_point();
    init->M6020_M2.gimbal_motor_measure = get_6020_M2_motor_measure_point();
    init->M6020_M3.gimbal_motor_measure = get_6020_M3_motor_measure_point();
    init->M6020_M4.gimbal_motor_measure = get_6020_M4_motor_measure_point();
    init->M3508_M1.chassis_motor_measure = get_3508_M1_motor_measure_point();
    init->M3508_M2.chassis_motor_measure = get_3508_M2_motor_measure_point();
    init->M3508_M3.chassis_motor_measure = get_3508_M3_motor_measure_point();
    init->M3508_M4.chassis_motor_measure = get_3508_M4_motor_measure_point();

    // 遥控器数据指针获取
    // init->rc_ctrl = get_remote_control_point();

    M6020_PID_init(&init->M6020_M1.gimbal_motor_relative_angle_pid, M6020_MOTOR_POSION_PID_MAX_OUT, M6020_MOTOR_POSION_PID_MAX_IOUT, M6020_MOTOR_POSION_PID_KP, M6020_MOTOR_POSION_PID_KI, M6020_MOTOR_POSION_PID_KD);
    PID_init(&init->M6020_M1.gimbal_motor_gyro_pid, PID_POSITION, M6020_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);

    M6020_PID_init(&init->M6020_M2.gimbal_motor_relative_angle_pid, M6020_MOTOR_POSION_PID_MAX_OUT, M6020_MOTOR_POSION_PID_MAX_IOUT, M6020_MOTOR_POSION_PID_KP, M6020_MOTOR_POSION_PID_KI, M6020_MOTOR_POSION_PID_KD);
    PID_init(&init->M6020_M2.gimbal_motor_gyro_pid, PID_POSITION, M6020_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);

    M6020_PID_init(&init->M6020_M3.gimbal_motor_relative_angle_pid, M6020_MOTOR_POSION_PID_MAX_OUT, M6020_MOTOR_POSION_PID_MAX_IOUT, M6020_MOTOR_POSION_PID_KP, M6020_MOTOR_POSION_PID_KI, M6020_MOTOR_POSION_PID_KD);
    PID_init(&init->M6020_M3.gimbal_motor_gyro_pid, PID_POSITION, M6020_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);

    M6020_PID_init(&init->M6020_M4.gimbal_motor_relative_angle_pid, M6020_MOTOR_POSION_PID_MAX_OUT, M6020_MOTOR_POSION_PID_MAX_IOUT, M6020_MOTOR_POSION_PID_KP, M6020_MOTOR_POSION_PID_KI, M6020_MOTOR_POSION_PID_KD);
    PID_init(&init->M6020_M4.gimbal_motor_gyro_pid, PID_POSITION, M6020_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);

    PID_init(&init->M3508_M1.chassis_motor_gyro_pid, PID_POSITION, M3508_speed_pid, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    PID_init(&init->M3508_M2.chassis_motor_gyro_pid, PID_POSITION, M3508_speed_pid, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    PID_init(&init->M3508_M3.chassis_motor_gyro_pid, PID_POSITION, M3508_speed_pid, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);
    PID_init(&init->M3508_M4.chassis_motor_gyro_pid, PID_POSITION, M3508_speed_pid, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);

    // 清除所有PID
    motor_total_pid_clear(init);

    motor_feedback_update(&motor_control);

    init->M6020_M1.relative_angle_set = init->M6020_M1.relative_angle;
    init->M6020_M2.relative_angle_set = init->M6020_M2.relative_angle;
    init->M6020_M3.relative_angle_set = init->M6020_M3.relative_angle;
    init->M6020_M4.relative_angle_set = init->M6020_M4.relative_angle;
    init->M3508_M1.motor_speed_set = init->M3508_M1.motor_speed;
    init->M3508_M2.motor_speed_set = init->M3508_M2.motor_speed;
    init->M3508_M3.motor_speed_set = init->M3508_M3.motor_speed;
    init->M3508_M4.motor_speed_set = init->M3508_M4.motor_speed;

}

// 编码值转为弧度制
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > 4096)
    {
        relative_ecd -= 8191;
    }
    else if (relative_ecd < -4096)
    {
        relative_ecd += 8191;
    }
    return relative_ecd * MOTOR_ECD_TO_RAD;
}

// 6020的PID初始化
static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

// 6020的PID计算
static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

// 清除6020的PID
static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear)
{
    if (M6020_pid_clear == NULL)
    {
        return;
    }

    M6020_pid_clear->err = M6020_pid_clear->set = M6020_pid_clear->get = 0.0f;
    M6020_pid_clear->out = M6020_pid_clear->Pout = M6020_pid_clear->Iout = M6020_pid_clear->Dout = 0.0f;
}

// 电机主控制循环
static void motor_control_loop(motor_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    // 计算所有6020电机pid
    M6020_motor_relative_angle_control(&control_loop->M6020_M1);
    M6020_motor_relative_angle_control(&control_loop->M6020_M2);
    M6020_motor_relative_angle_control(&control_loop->M6020_M3);
    M6020_motor_relative_angle_control(&control_loop->M6020_M4);
    CAN_cmd_6020(motor_control.M6020_M1.given_current, motor_control.M6020_M2.given_current, motor_control.M6020_M3.given_current, motor_control.M6020_M4.given_current);

    // 计算所有3508电机pid
    M3508_motor_speed_control(&control_loop->M3508_M1);
    M3508_motor_speed_control(&control_loop->M3508_M2);
    M3508_motor_speed_control(&control_loop->M3508_M3);
    M3508_motor_speed_control(&control_loop->M3508_M4);
    // 发送给电机数据
    CAN_cmd_3508(motor_control.M3508_M1.given_current, motor_control.M3508_M2.given_current, motor_control.M3508_M3.given_current, motor_control.M3508_M4.given_current);
}

// 6020的pid计算
static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    int flag = 1;
    // 角度环，速度环串级pid

    gimbal_motor->motor_gyro_set = M6020_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set); // 控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

// 3508的pid计算
static void M3508_motor_speed_control(motor_3508_t *chassis_motor)
{
    if (chassis_motor == NULL)
    {
        return;
    }

    // 速度环pid
    chassis_motor->current_set = PID_calc(&chassis_motor->chassis_motor_gyro_pid, chassis_motor->motor_speed, chassis_motor->motor_speed_set); // 控制值赋值
    chassis_motor->given_current = (int16_t)(chassis_motor->current_set);

}

// 各个电机的数据反馈
static void motor_feedback_update(motor_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    // 数据更新,各个航向电机的角度数据
    feedback_update->M6020_M1.relative_angle = motor_ecd_to_angle_change(feedback_update->M6020_M1.gimbal_motor_measure->ecd,
                                                                         feedback_update->M6020_M1.offset_ecd);
    feedback_update->M6020_M2.relative_angle = motor_ecd_to_angle_change(feedback_update->M6020_M2.gimbal_motor_measure->ecd,
                                                                         feedback_update->M6020_M2.offset_ecd);
    feedback_update->M6020_M3.relative_angle = motor_ecd_to_angle_change(feedback_update->M6020_M3.gimbal_motor_measure->ecd,
                                                                         feedback_update->M6020_M3.offset_ecd);
    feedback_update->M6020_M4.relative_angle = motor_ecd_to_angle_change(feedback_update->M6020_M4.gimbal_motor_measure->ecd,
                                                                         feedback_update->M6020_M4.offset_ecd);
    // 数据更新,各个航向电机的角度数据

    // 数据更新,各个动力电机的速度数据
    feedback_update->M3508_M1.motor_speed = feedback_update->M3508_M1.chassis_motor_measure->speed_rpm;
    feedback_update->M3508_M2.motor_speed = feedback_update->M3508_M2.chassis_motor_measure->speed_rpm;
    feedback_update->M3508_M3.motor_speed = feedback_update->M3508_M3.chassis_motor_measure->speed_rpm;
    feedback_update->M3508_M4.motor_speed = feedback_update->M3508_M4.chassis_motor_measure->speed_rpm;

    feedback_update->chassis_speed_avg = (feedback_update->M3508_M1.motor_speed + feedback_update->M3508_M2.motor_speed + feedback_update->M3508_M3.motor_speed + feedback_update->M3508_M4.motor_speed) / 4.0f;
    // 数据更新,各个动力电机的速度数据
}

// 运动计算
static void movement_calc(void)
{
    // 此处为遥控器控制的方式
    float theta = atan(21.5 / 41.5);
    chassis_float.speed = (float)hypot(rc_ctrl.rc.ch[2] / 66 * 400, rc_ctrl.rc.ch[3] / 66 * 400);
    chassis_float.angle = (float)atan2(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]);
    chassis_float.W = (float)(rc_ctrl.rc.ch[4] / 66 * 200);

    motor_control.M3508_M1.motor_speed_set = chassis_float.speed - chassis_float.W;
    motor_control.M3508_M2.motor_speed_set = -(chassis_float.speed + chassis_float.W);
    motor_control.M3508_M3.motor_speed_set = chassis_float.speed - chassis_float.W;
    motor_control.M3508_M4.motor_speed_set = -(chassis_float.speed + chassis_float.W);
    if (fabs(chassis_float.W) < 10)
    {
        motor_control.M6020_M1.relative_angle_set = chassis_float.angle + 6144 * MOTOR_ECD_TO_RAD;
        motor_control.M6020_M2.relative_angle_set = chassis_float.angle + 3413 * MOTOR_ECD_TO_RAD;
        motor_control.M6020_M3.relative_angle_set = chassis_float.angle + 7509 * MOTOR_ECD_TO_RAD;
        motor_control.M6020_M4.relative_angle_set = chassis_float.angle + 2048 * MOTOR_ECD_TO_RAD;
    }

    else if (fabs(chassis_float.W) > 10)
    {

        motor_control.M6020_M1.relative_angle_set = chassis_float.angle + 6144 * MOTOR_ECD_TO_RAD + theta;
        motor_control.M6020_M2.relative_angle_set = chassis_float.angle + 3413 * MOTOR_ECD_TO_RAD - theta;
        motor_control.M6020_M3.relative_angle_set = chassis_float.angle + 7509 * MOTOR_ECD_TO_RAD - theta;
        motor_control.M6020_M4.relative_angle_set = chassis_float.angle + 2048 * MOTOR_ECD_TO_RAD + theta;
    }
    // vofa调试用的代码
    // uart_dma_printf(&huart1,"%4.3f ,%4.3f\n",set_speed , set_angle);
}

//自动模式运动计算
static void movement_calc_auto(void)
{
    // 此处为遥控器控制的方式
    float theta = atan(21.5 / 41.5);
    chassis_float.speed = chassis_auto_move_cmd_data.speed;
    chassis_float.angle = chassis_auto_move_cmd_data.angle;
    //chassis_float.W = (float)(rc_ctrl.rc.ch[4] / 66 * 200);

    chassis_float.W = 0;
    motor_control.M3508_M1.motor_speed_set = chassis_float.speed - chassis_float.W;
    motor_control.M3508_M2.motor_speed_set = -(chassis_float.speed + chassis_float.W);
    motor_control.M3508_M3.motor_speed_set = chassis_float.speed - chassis_float.W;
    motor_control.M3508_M4.motor_speed_set = -(chassis_float.speed + chassis_float.W);
    if (fabs(chassis_float.W) < 10)
    {
        
        motor_control.M6020_M1.relative_angle_set = chassis_float.angle + 6144 * MOTOR_ECD_TO_RAD;
        motor_control.M6020_M2.relative_angle_set = chassis_float.angle + 3413 * MOTOR_ECD_TO_RAD;
        motor_control.M6020_M3.relative_angle_set = chassis_float.angle + 7509 * MOTOR_ECD_TO_RAD;
        motor_control.M6020_M4.relative_angle_set = chassis_float.angle + 2048 * MOTOR_ECD_TO_RAD;
    }

    else if (fabs(chassis_float.W) > 10)
    {  

        motor_control.M6020_M1.relative_angle_set = chassis_float.angle + 6144 * MOTOR_ECD_TO_RAD + theta;
        motor_control.M6020_M2.relative_angle_set = chassis_float.angle + 3413 * MOTOR_ECD_TO_RAD - theta;
        motor_control.M6020_M3.relative_angle_set = chassis_float.angle + 7509 * MOTOR_ECD_TO_RAD - theta;
        motor_control.M6020_M4.relative_angle_set = chassis_float.angle + 2048 * MOTOR_ECD_TO_RAD + theta;

    }
    // vofa调试用的代码
    // uart_dma_printf(&huart1,"%4.3f ,%4.3f\n",set_speed , set_angle);
    //uart_dma_printf(&huart1, "%4.3f, %4.3f\n" , chassis_auto_move_cmd_data.speed , chassis_auto_move_cmd_data.angle);
}

/**
  * @brief  找出两角的较小差值
  * @param  角1，角2
  * @retval 
  * @attention 
  */
fp32 Find_min_Angle(int16_t angle1,fp32 angle2)
{
	  fp32 err;
    err = (fp32)angle1 - angle2;
    if(fabs(err) > 4096)
    {
        err = 8192 - fabs(err);
    }
    return err;
}


void AngleLoop_f (float* angle ,float max){
	while((*angle<-(max/2)) ||(*angle>(max/2)))
	{
		if(*angle<-(max/2))
		{
			*angle+=max;
		}
		else if(*angle>(max/2))
		{
			*angle-=max;
		}
  }
}