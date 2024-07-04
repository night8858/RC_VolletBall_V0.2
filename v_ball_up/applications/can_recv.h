#ifndef CAN_RECV_H
#define CAN_RECV_H

#include "struct_typedef.h"
#include "can.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan1

#define P_MIN -12.5f // 位置最小值
#define P_MAX 12.5f  // 位置最大值
#define V_MIN -10.0f   // 速度最小值
#define V_MAX 10.0f    // 速度最大值
#define KP_MIN 0.0f    // Kp最小值
#define KP_MAX 500.0f  // Kp最大值
#define KD_MIN 0.0f    // Kd最小值
#define KD_MAX 5.0f    // Kd最大值
#define T_MIN -28.0f   // 转矩最大值
#define T_MAX 28.0f    // 转矩最小值

/* CAN send and receive ID */
typedef enum
{
    CAN_3508_5_8_ID = 0x1FF,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M5_ID = 0x205,

    DT7_RX_ch = 0x301,
    DT7_RX_S = 0x302,
    AUTO_MODE_CMD = 0x303,

    DM4340_M1 = 0x11,
    DM4340_M2 = 0x12,
    DM4340_M3 = 0x13,

} can_msg_id_e;

//dbus控制器的数据结构体
typedef struct
{
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t dial;
    uint8_t switch_left;            /*   1    */
    uint8_t switch_left_last;       /*   3    */
    uint8_t switch_right;		    /*   2    */
    uint8_t switch_right_last;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_left;
        uint8_t press_right;

        uint8_t press_left_hop;//鼠标左键跳变
        uint8_t press_right_hop;//鼠标右键跳变
    } mouse;
    struct
    {
        uint16_t key;
        uint16_t key_hop; //跳变键值
    } keyBoard;
} DBUSDecoding_Type;


typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float esc_back_position; // 返回的位置-12.5~12.5
    float esc_back_speed;    // 反馈速度-45.0~45.0
    float esc_back_current;  // 反馈电流/扭矩
    float esc_back_angle;    // 反馈电流/扭矩
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    /*处理连续码盘值*/
    float esc_back_position_last; // 上一次返回的位置
    int64_t circle_num;           // 旋转圈数
    float serial_position;        // 总码盘值
    float serial_angle;           // 总角度
    float serial_angle_last;
    float real_angle; // 真实角度

    /*目标值*/
    float target_speed; // 目标速度
    float set_speed;
    double target_position; // 目标位置
    float target_angle;     // 目标角度
    int target_state;

    /*电机输出电流*/
    float out_current; // 输出电流

} DM4340_motor_data_t;

// rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

void CAN_cmd_3508(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4);

extern void MD_CanReceive(DM4340_motor_data_t *motor, uint8_t RxDate[8]);
void Byte_to_Float(float *fx, float *fy, float *fz, float *fw, unsigned char byte[]);
void MD_motor_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
float RUD_DirAngle_Proc(float Angle);
int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
static float uint_to_float(int x_int, float x_min, float x_max, int bits);
void chassis_cmd_aotu(CAN_HandleTypeDef *Target_hcan);
void start_motor(CAN_HandleTypeDef *Target_hcan, uint16_t id);

void Float_to_Byte(float a, float b, unsigned char byte[]);

extern const motor_measure_t *get_3508_M5_motor_measure_point(void);
const DM4340_motor_data_t *get_4340_M1_motor_measure_point(void);
const DM4340_motor_data_t *get_4340_M2_motor_measure_point(void);
const DM4340_motor_data_t *get_4340_M3_motor_measure_point(void);

#endif
