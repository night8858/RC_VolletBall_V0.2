#include "main.h"
#include "top_ctrl.h"
#include "math.h"
#include "pid.h"
#include "can.h"
#include "can_recv.h"
#include "chassis.h"
#include "usart.h"
#include "bsp_usart.h"
#include "ball_track.h"
#include "struct_typedef.h"
#include "user_lib.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

fp32 filter_coefficient[1];

extern DM4340_motor_data_t DM4340_Date[3];    // 引用DM4340回传数据结构体
extern motor_control_t motor_control;         // 引用底盘电机控制结构体
extern DBUSDecoding_Type DBUS_ReceiveData;    // 底盘发来的dbus的数据
extern ball_track_target_t ball_track_target; // 球追踪目标点

double T[3];     // T是引入万能公式的一个变量。令tan（THETA/2） = T,这样就可以将式子中相关与THETA的复杂运算给简化成用T表示。
double THETA[3]; // THETA变量表示三个电机的角度，可由万能公式的T解出

float angle[3];

Top_Pos delta_position;
Ball_Pos RX_ball_pos;
first_order_filter_type_t filter_angle;

double A[3] = {0};
double B[3] = {0};
double C[3] = {0};

double K[3] = {0};
double U[3] = {0};
double V[3] = {0};

// 球拍控制主循环
void top_contorl_Task(void const *argument)
{
    filter_coefficient[0] = 0.6;
    DM_Motor_Init();
    motor_init(&motor_control);
    // M3508_M5_Pos_init();
    ball_track_pid_init();
    ball_track_target.frist_hit_flag = 0;   //起颠标志位
    int date_zero_flag = 0;   //数据清零标志位
    ball_track_target.hit_num = 0;
    // first_order_filter_init(&filter_angle , 50 , filter_coefficient);
    while (1)
    {
       
        osDelay(2);
        uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f ,%4.3f, %4.3f , %3.3f\n",
                        RX_ball_pos.ball_pos_x,
                        ball_track_target.speed,
                        ball_track_target.motor_real_speed,
                        ball_track_target.ball_speed,
                        ball_track_target.offset_pos,
                        ball_track_target.real_distance);
                        

        ball_track_target.mode_falg = 0;
        //  此处为手动操作模式
        if (DBUS_ReceiveData.switch_left == 1 && DBUS_ReceiveData.switch_right == 1)
        {
             ball_track_target.mode_falg = 1;
            ball_track_target.frist_hit_flag = 0;
            date_zero_flag = 0; 

            juggle_Mode();
            // Institution_Pos_Contorl();
            //osDelay(2);
        }

        // 此处为自动模式
        if (DBUS_ReceiveData.switch_left == 3 && DBUS_ReceiveData.switch_right == 3)
        {   
            if (date_zero_flag == 0)
            {
               ball_track_target.speed = 0;
               ball_track_target.offset_pos = 0;
               ball_track_target.angle = 0;
               date_zero_flag = 1;
            }
            
            if (ball_track_target.frist_hit_flag == 0)
            {
                osDelay(200);  //此处为颠球启动程序，
                hit_start();
                
                osDelay(100);
                //ball_track_target.frist_hit_flag = 1;
            }

            //ball_track_calc();
            juggle_Mode_auto();
            // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n",RX_ball_pos.ball_pos_x, RX_ball_pos.ball_pos_y, RX_ball_pos.ball_pos_z);
            //osDelay(2);
        }
        // 复位
        if (DBUS_ReceiveData.switch_left != DBUS_ReceiveData.switch_right)
        {
            ball_track_target.mode_falg = 0;
            ball_track_target.frist_hit_flag = 0;
            ball_track_target.hit_flag = 0;
            ball_track_target.hit_num = 0;
             HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
            date_zero_flag =0;
            DM_Motor_Init();
            motor_init(&motor_control);
            ball_track_pid_init();
            //osDelay(2);
        }
    }
}

// 达妙电机初始化
void DM_Motor_Init(void)
{

    for (int i = 0; i < 3; i++)
    {
        start_motor(&hcan2, 0x01);
        osDelay(300);
        start_motor(&hcan2, 0x02);
        osDelay(300);
        start_motor(&hcan2, 0x03);
        osDelay(300);
    }
    DM4340_Date[0].target_angle = (-(float_constrain(83, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 颠球模式
static void juggle_Mode(void)
{
    int i = 0;
    if (DBUS_ReceiveData.ch1 != 0)
    {
        i = 22;
    }

    DM4340_Date[0].target_angle = (-(float_constrain(83 + i, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + i, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + i, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    // vofa测试代码，可注释

    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[0].target_angle / PI * 180, DM4340_Date[0].out_current);
}

// 测试模式
static void test_Mode(void)
{

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, 0, 0, 0);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, 0, 0, 0);
    osDelay(2);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, 0, 0, 0);
    osDelay(2);
    angle[0] = DM4340_Date[0].real_angle;
    angle[1] = DM4340_Date[1].real_angle;
    angle[2] = DM4340_Date[2].real_angle;
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[0].target_angle / PI * 180, DM4340_Date[0].out_current);
}

// 有、bug，不用
static void resolving_Mode(void)
{
    delta_position.x = DBUS_ReceiveData.ch2 / 66;
    delta_position.y = DBUS_ReceiveData.ch3 / 66;
    delta_position.z = -DBUS_ReceiveData.ch1 / 330;

    delta_arm_solution();

    // DM4340_Date[0].target_angle = (-(float_constrain( (103-THETA[0]), 73, 113)) / 180 * PI);
    // DM4340_Date[1].target_angle = (-(float_constrain( (167-THETA[1]), 137, 177)) / 180 * PI);
    // DM4340_Date[2].target_angle = (-(float_constrain( (159-THETA[2]), 129, 169)) / 180 * PI);

    DM4340_Date[0].target_angle = -(float_constrain((65 + THETA[0]), 73, 113));
    DM4340_Date[1].target_angle = -(float_constrain((133 + THETA[1]), 137, 177));
    DM4340_Date[2].target_angle = -(float_constrain((118 + THETA[2]), 129, 169));

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[0].target_angle / PI * 180, DM4340_Date[0].out_current);
}

// 自动颠球模式
static void juggle_Mode_auto(void)
{
    //uint16_t angle = 0;
    if (ball_track_target.hit_flag == 1)
    {
        hit_once();
        ball_track_target.hit_num ++;
        //angle = 22;
        //// HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
        //if (ball_track_target.hit_flag == 2)
        //{
        //    angle = 30;
        //}
    }
    if (ball_track_target.hit_flag == 2)
    {
        hit_big_once();
        back_to_zero();
        //angle = 0;
        // HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
    }
    if(ball_track_target.hit_flag == 0)
    {

        back_to_zero();
    }
    //DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    //DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    //DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    //MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    //osDelay(1);
    //MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    //osDelay(1);
    //MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    //osDelay(1);
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
}

static void back_to_zero(void)
{
    uint16_t angle = 0;

    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(2);
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
}

static void hit_once(void)
{
    uint16_t angle = 16;   //21.6

    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(200);

    angle = 0;

    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(200);
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
}

static void hit_big_once(void)
{
    uint16_t angle = 28;

    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(200);
    // vofa测试代码，可注释
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
}


// 启动颠球
static void hit_start(void)
{
    uint16_t angle = 25;

    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);

    osDelay(180);

    angle = 0;
    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    // vofa测试代码，可注释
    ball_track_target.frist_hit_flag = 1;
    osDelay(100);
    
    // uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n", DM4340_Date[0].real_angle, DM4340_Date[1].real_angle, DM4340_Date[2].real_angle);
    
/*
    angle = 30;
    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);

    osDelay(180);

    angle = 0;
    DM4340_Date[0].target_angle = (-(float_constrain(83 + angle, 83, 113)) / 180 * PI);
    DM4340_Date[1].target_angle = (-(float_constrain(147 + angle, 147, 177)) / 180 * PI);
    DM4340_Date[2].target_angle = (-(float_constrain(139 + angle, 139, 169)) / 180 * PI);

    MD_motor_SendCurrent(&hcan2, 1, DM4340_Date[0].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 2, DM4340_Date[1].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    MD_motor_SendCurrent(&hcan2, 3, DM4340_Date[2].target_angle, 0, DM_MOTOR_KP, DM_MOTOR_KD, DM_MOTOR_t_ff);
    osDelay(1);
    ball_track_target.frist_hit_flag = 1;
    // vofa测试代码，可注释
    osDelay(200);‘*/
}

float RUD_DirAngle_c(float Angle)
{
    while (Angle > 18000 || Angle < 0)
    {
        if (Angle < 0)
        {
            Angle += 360;
        }
        if (Angle > 360)
        {
            Angle -= 360;
        }
    }
    return (float)Angle;
}

// 球拍姿态解算
void delta_arm_solution(void)
{

    A[0] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) - 2 * delta_position.x * (R1 - R2)) / (2 * L1);
    B[0] = -(R1 - R2 - delta_position.x);
    C[0] = delta_position.z;

    A[1] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (delta_position.x - sqrt(3) * delta_position.y) * (R1 - R2)) / L1;
    B[1] = -2 * (R1 - R2) - (delta_position.x - sqrt(3) * delta_position.y);
    C[1] = 2 * delta_position.z;

    A[2] = (delta_position.x * delta_position.x + delta_position.y * delta_position.y + delta_position.z * delta_position.z + L1 * L1 - La * La + (R1 - R2) * (R1 - R2) + (delta_position.x - sqrt(3) * delta_position.y) * (R1 - R2)) / L1;
    B[2] = -2 * (R1 - R2) - (delta_position.x + sqrt(3) * delta_position.y);
    C[2] = 2 * delta_position.z;

    K[0] = A[0] + B[0];
    U[0] = 2 * C[0];
    V[0] = A[0] - B[0];

    K[1] = A[1] + B[1];
    U[1] = 2 * C[1];
    V[1] = A[1] - B[1];

    K[2] = A[2] + B[2];
    U[2] = 2 * C[2];
    V[2] = A[2] - B[2];

    T[0] = (-U[0] - sqrt(U[0] * U[0] - 4 * K[0] * V[0])) / (2 * K[0]);
    T[1] = (-U[1] - sqrt(U[1] * U[1] - 4 * K[1] * V[1])) / (2 * K[1]);
    T[2] = (-U[2] - sqrt(U[2] * U[2] - 4 * K[2] * V[2])) / (2 * K[2]);

    //	THETA[0] = (180 * (2 * atan(T[0]))) / PI;
    //	THETA[1] = (180 * (2 * atan(T[1]))) / PI;
    //	THETA[2] = (180 * (2 * atan(T[2]))) / PI;

    THETA[0] = (180 * (2 * atan(T[1]))) / PI;
    THETA

    [1] = (180 * (2 * atan(T[2]))) / PI;
    THETA[2] = (180 * (2 * atan(T[0]))) / PI;
}

//// pos的PID初始化
// static void POS_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
//{
//     if (pid == NULL)
//     {
//         return;
//     }
//     pid->kp = kp;
//     pid->ki = ki;
//     pid->kd = kd;
//
//     pid->err = 0.0f;
//     pid->get = 0.0f;
//
//     pid->max_iout = max_iout;
//     pid->max_out = maxout;
// }
//
//// 6020的PID计算
// static fp32 POS_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
//{
//     fp32 err;
//     if (pid == NULL)
//     {
//         return 0.0f;
//     }
//     pid->get = get;
//     pid->set = set;
//
//     err = set - get;
//     pid->err = rad_format(err);
//     pid->Pout = pid->kp * pid->err;
//     pid->Iout += pid->ki * pid->err;
//     pid->Dout = pid->kd * error_delta;
//     abs_limit(&pid->Iout, pid->max_iout);
//     pid->out = pid->Pout + pid->Iout + pid->Dout;
//     abs_limit(&pid->out, pid->max_out);
//     return pid->out;
// }

//延时函数
void delay(int count)
{
	int i;
	for(i=1;i<=count;i++)
	;
}
