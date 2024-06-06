#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "chassis.h"
#include "can_recv.h"
#include "user_lib.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "math.h"

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

extern motor_measure_t motor_Date[8];
motor_control_t motor_control;

extern DBUSDecoding_Type DBUS_ReceiveData;

extern RC_ctrl_t rc_ctrl;
extern UART_HandleTypeDef huart1;

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define motor_total_pid_clear(motor_clear)                                        \
    {                                                                             \
        PID_clear(&(motor_clear)->M3508_M5.chassis_motor_gyro_pid);               \
        M3508_PID_clear(&(motor_clear)->M3508_M5.m3508_motor_relative_angle_pid); \
    }

static void motor_feedback_update(motor_control_t *feedback_update);
// static void M3508_motor_speed_control(motor_3508_t *chassis_motor);
static void motor_control_loop(motor_control_t *control_loop);
static void motor_feedback_update(motor_control_t *feedback_update);
// static void movement_calc(void);

// 地盘主线程，颠球机构未使用
void chassisTask(void const *argument)
{
    // vTaskDelay(GIMBAL_TASK_INIT_TIME);
    motor_init(&motor_control);
    while (1)
    {
        // 取得回传数据结构体
        motor_feedback_update(&motor_control);
        // movement_calc();
        //  获取遥控器数据结构体测试
        //  motor_control.M6020_M1.relative_angle_set = (float)rc_ctrl.rc.ch[2] / 660 * PI;  //-pi到pi
        //  motor_control.M3508_M5.motor_speed_set = (float)rc_ctrl.rc.ch[1] / 66 * 400;    // -4096到+4096
        //  PID控制计算和输出循环
        motor_control_loop(&motor_control);
        // uart_dma_printf(&huart1,"%4.3f ,%4.3f\n",motor_control.M6020_M1.relative_angle , motor_control.M6020_M1.relative_angle_set);
        // uart_dma_printf(&huart1,"%4.3f ,%4.3f , %4.3f\n",motor_control.M3508_M5.motor_speed / 19, motor_control.M3508_M5.motor_speed_set / 19 , motor_control.M3508_M5.motor_speed - motor_control.M3508_M5.motor_speed_set);

        osDelay(2);
    }
}

// 电机数据的初始化
void motor_init(motor_control_t *init)
{

    const fp32 M3508_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    // 电机数据指针获取
    init->M3508_M5.chassis_motor_measure = get_3508_M5_motor_measure_point();

    // 3508PID的初始化
    M3508_Pos_PID_init(&init->M3508_M5.m3508_motor_relative_angle_pid, M3508_MOTOR_POSION_PID_MAX_OUT, M3508_MOTOR_POSION_PID_MAX_IOUT, M3508_MOTOR_POSION_PID_KP, M3508_MOTOR_POSION_PID_KI, M3508_MOTOR_POSION_PID_KD);
    PID_init(&init->M3508_M5.chassis_motor_gyro_pid, PID_POSITION, M3508_speed_pid, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);

    // 清除所有PID
    motor_total_pid_clear(init);

    motor_feedback_update(&motor_control);

    init->M3508_M5.motor_speed_set = init->M3508_M5.motor_speed;
    init->M3508_M5.motor_ecd = 0;
    init->M3508_M5.self_ecd_temp = 0;
    init->M3508_M5.serial_angle = 0;
    init->M3508_M5.relative_angle_target = 0;
    // init->M3508_M5.self_ecd_temp += init->M3508_M5.esc_back_position;
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

// 3508的PID初始化
static void M3508_Pos_PID_init(M3508_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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

// 3508的PID计算
static fp32 M3508_Pos_PID_calc(M3508_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

// 清除3508的PID
static void M3508_PID_clear(M3508_PID_t *M3508_pid_clear)
{
    if (M3508_pid_clear == NULL)
    {
        return;
    }

    M3508_pid_clear->err = M3508_pid_clear->set = M3508_pid_clear->get = 0.0f;
    M3508_pid_clear->out = M3508_pid_clear->Pout = M3508_pid_clear->Iout = M3508_pid_clear->Dout = 0.0f;
}

// 3508的pid计算
void M3508_motor_Pos_control(motor_3508_t *chassis_motor)
{
    if (chassis_motor == NULL)
    {
        return;
    }

    // 速度环pid
    chassis_motor->motor_speed_set = M3508_Pos_PID_calc(&chassis_motor->m3508_motor_relative_angle_pid, chassis_motor->serial_angle, chassis_motor->relative_angle_target, chassis_motor->motor_gyro); // 控制值赋值
    chassis_motor->current_set = PID_calc(&chassis_motor->chassis_motor_gyro_pid, chassis_motor->motor_speed, chassis_motor->motor_speed_set);                                                         // 控制值赋值
    chassis_motor->given_current = (int16_t)(chassis_motor->current_set);
    // chassis_motor->given_current = (int16_t)(chassis_motor->current_set);
}

// 顶部3508电机的编码器初始化
void M3508_motor_Pos_init(motor_3508_t *chassis_motor)
{
    if (chassis_motor == NULL)
    {
        return;
    }

    // 速度环pid
    // chassis_motor->motor_speed_set = M3508_Pos_PID_calc(&chassis_motor->m3508_motor_relative_angle_pid , chassis_motor->serial_angle ,chassis_motor->relative_angle_target ,chassis_motor->motor_gyro); // 控制值赋值
    chassis_motor->current_set = PID_calc(&chassis_motor->chassis_motor_gyro_pid, chassis_motor->motor_speed, chassis_motor->motor_speed_set); // 控制值赋值
    chassis_motor->given_current = (int16_t)(chassis_motor->current_set);
    CAN_cmd_3508(motor_control.M3508_M5.given_current, 0, 0, 0);
}

// 电机主控制循环
void motor_control_loop(motor_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    // uint16_t Counting_mark = 0; // 计数标记
    //  计算所有3508电机pid
    motor_control.M3508_M5.relative_angle_target = motor_control.M3508_M5.serial_angle + DBUS_ReceiveData.ch0 * 10;
    M3508_motor_Pos_control(&control_loop->M3508_M5);
    // 发送给电机数据
    CAN_cmd_3508(motor_control.M3508_M5.given_current, 0, 0, 0);
}

// 各个电机的数据反馈
void motor_feedback_update(motor_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    int Counting_mark = 0;                                                                              // 计数标记
    feedback_update->M3508_M5.motor_speed = feedback_update->M3508_M5.chassis_motor_measure->speed_rpm; // 获得当前的转速
    feedback_update->M3508_M5.esc_back_position_last = feedback_update->M3508_M5.esc_back_position;     // 获得上一次的角度
    feedback_update->M3508_M5.esc_back_position = feedback_update->M3508_M5.chassis_motor_measure->ecd; // 获得当前角度

    Counting_mark = feedback_update->M3508_M5.esc_back_position - feedback_update->M3508_M5.esc_back_position_last;
    if (Counting_mark > 0)
    {
        if (Counting_mark < 4096)
        {
            feedback_update->M3508_M5.serial_angle += Counting_mark;
        }
        if (Counting_mark > 4096)
        {

            feedback_update->M3508_M5.serial_angle += Counting_mark;
            feedback_update->M3508_M5.serial_angle -= 8192;
        }
    }
    if (Counting_mark < 0)
    {
        if (Counting_mark > -4096)
        {
            feedback_update->M3508_M5.serial_angle += Counting_mark;
        }
        if (Counting_mark < -4096)
        {
            feedback_update->M3508_M5.serial_angle += Counting_mark;
            feedback_update->M3508_M5.serial_angle += 8192;
        }
    }
    // feedback_update->M3508_M5.serial_angle = feedback_update->M3508_M5.self_ecd_temp;
    //  数据更新,各个动力电机的速度数据
}

// 机构横向运动
void Institution_Pos_Contorl(void)
{

    // 取得回传数据结构体
    motor_feedback_update(&motor_control);

    // movement_calc();
    //  PID控制计算和输出循环
    motor_control_loop(&motor_control);
    // vofa调试用的代码
    // uart_dma_printf(&huart1, " %0.0f , %0.0f\n", motor_control.M3508_M5.relative_angle_target, motor_control.M3508_M5.serial_angle );//, motor_control.M3508_M5.raw_cmd_current ,motor_control.M3508_M5.current_set);
    osDelay(5);
}

// 机构编码器数据更新的初始化
void M3508_M5_Pos_init(void)
{
    uint8_t init_flag = 0;
    uint8_t stop_flag = 0;
    while (1)
    {
        // 取得回传数据结构体
        motor_feedback_update(&motor_control);
        motor_control.M3508_M5.motor_speed_set = -1024;
        M3508_motor_Pos_init(&motor_control.M3508_M5);
        osDelay(3);
        if (motor_control.M3508_M5.esc_back_position == motor_control.M3508_M5.esc_back_position_last)
        {
            stop_flag++;
            if ( stop_flag > 100)
            {
                init_flag = 1;
            }
            
        }
        
        if (init_flag == 1){return;}

    }
    
    motor_control.M3508_M5.serial_angle = 0;
    // 机构编码器数据更新的初始化
}