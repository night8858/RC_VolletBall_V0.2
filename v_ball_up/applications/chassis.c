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

#define motor_total_pid_clear(motor_clear)                          \
    {                                                               \
        PID_clear(&(motor_clear)->M3508_M5.chassis_motor_gyro_pid); \
    }                                   

// void motor_init(motor_control_t *init);
static void motor_feedback_update(motor_control_t *feedback_update);
static void M3508_motor_speed_control(motor_3508_t *chassis_motor);
static void motor_control_loop(motor_control_t *control_loop);
static void motor_feedback_update(motor_control_t *feedback_update);
static void movement_calc(void);

// 地盘主线程，颠球机构未使用
void chassisTask(void const *argument)
{
    // vTaskDelay(GIMBAL_TASK_INIT_TIME);
    motor_init(&motor_control);
    while (1)
    {
        // 取得回传数据结构体
        motor_feedback_update(&motor_control);
        movement_calc();
        // 获取遥控器数据结构体测试
        // motor_control.M6020_M1.relative_angle_set = (float)rc_ctrl.rc.ch[2] / 660 * PI;  //-pi到pi
        // motor_control.M3508_M5.motor_speed_set = (float)rc_ctrl.rc.ch[1] / 66 * 400;    // -4000到+4000
        // PID控制计算和输出循环
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

    // static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    // 电机数据指针获取

    init->M3508_M5.chassis_motor_measure = get_3508_M5_motor_measure_point();

    // 遥控器数据指针获取
    // init->rc_ctrl = get_remote_control_point();

    PID_init(&init->M3508_M5.chassis_motor_gyro_pid, PID_POSITION, M3508_speed_pid, M3505_MOTOR_SPEED_PID_MAX_IOUT, M3505_MOTOR_SPEED_PID_MAX_OUT);

    // 清除所有PID
    motor_total_pid_clear(init);

    motor_feedback_update(&motor_control);

    init->M3508_M5.motor_speed_set = init->M3508_M5.motor_speed;

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

// 电机主控制循环
void motor_control_loop(motor_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    // 计算所有3508电机pid
    M3508_motor_speed_control(&control_loop->M3508_M5);

    // 发送给电机数据
    CAN_cmd_3508(motor_control.M3508_M5.given_current, 0 , 0 , 0);
}

// 3508的pid计算
void M3508_motor_speed_control(motor_3508_t *chassis_motor)
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
void motor_feedback_update(motor_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    feedback_update->M3508_M5.motor_speed = feedback_update->M3508_M5.chassis_motor_measure->speed_rpm;
    // 数据更新,各个动力电机的速度数据
}

// 运动计算
void movement_calc(void)
{
    // 此处为遥控器控制的方式
    motor_control.M3508_M5.motor_speed_set = DBUS_ReceiveData.ch0/66 * 20;
    // vofa调试用的代码
    // uart_dma_printf(&huart1,"%4.3f ,%4.3f\n",set_speed , set_angle);
}


//机构横向运动
void Institution_Pos_Contorl(void)
{

        // 取得回传数据结构体
        motor_feedback_update(&motor_control);
        movement_calc();
        // PID控制计算和输出循环
        motor_control_loop(&motor_control);

}
