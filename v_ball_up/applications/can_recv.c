#include "main.h"
#include "can_recv.h"
#include "math.h"
#include "chassis.h"
#include "ball_track.h"
#include "top_ctrl.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern ball_track_target_t ball_track_target; // 球追踪目标点

static motor_measure_t motor_Date[7];         // 电机回传数据结构体
DM4340_motor_data_t DM4340_Date[3];           // DM4340回传数据结构体
static CAN_TxHeaderTypeDef RM3508_tx_message; // can_3508发送邮箱

CAN_TxHeaderTypeDef CAN_DMstart_TxHeader;
CAN_TxHeaderTypeDef CAN_DMmsg_TxHeader;

DBUSDecoding_Type DBUS_ReceiveData;

static uint8_t can_3508_send_data[8];

int DM_circle_num[4] = {0}; // 达妙电机圈数计数

#define get_motor_measure(ptr, data)                                   \
    {                                                                  \
        (ptr)->last_ecd = (ptr)->ecd;                                  \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->temperate = (data)[6];                                  \
    }

// 所有3508电机的指令发送
void CAN_cmd_3508(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4)
{
    uint32_t send_mail_box;
    RM3508_tx_message.StdId = CAN_3508_5_8_ID;
    RM3508_tx_message.IDE = CAN_ID_STD;
    RM3508_tx_message.RTR = CAN_RTR_DATA;
    RM3508_tx_message.DLC = 0x08;
    can_3508_send_data[0] = (CMD_ID_1 >> 8);
    can_3508_send_data[1] = CMD_ID_1;
    can_3508_send_data[2] = (CMD_ID_2 >> 8);
    can_3508_send_data[3] = CMD_ID_2;
    can_3508_send_data[4] = (CMD_ID_3 >> 8);
    can_3508_send_data[5] = CMD_ID_3;
    can_3508_send_data[6] = (CMD_ID_4 >> 8);
    can_3508_send_data[7] = CMD_ID_4;
    HAL_CAN_AddTxMessage(&hcan1, &RM3508_tx_message, can_3508_send_data, &send_mail_box);
}

// 电机启动函数
void start_motor(CAN_HandleTypeDef *Target_hcan, uint16_t id)
{
    // uint32_t send_mail_box;
    CAN_TxHeaderTypeDef CAN_TxHeader;
    uint8_t TxData[8];
    CAN_TxHeader.StdId = id;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = 0x08;
    TxData[0] = 0xFF;
    TxData[1] = 0xFF;
    TxData[2] = 0xFF;
    TxData[3] = 0xFF;
    TxData[4] = 0xFF;
    TxData[5] = 0xFF;
    TxData[6] = 0xFF;
    TxData[7] = 0xFC;

    HAL_CAN_AddTxMessage(Target_hcan, &CAN_TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX0);
    // HAL_CAN_AddTxMessage(Target_hcan, &CAN_DMstart_TxHeader, TxData, (uint32_t *)CAN_TX_MAILBOX0);
}

// auto_mode下底盘电机指令发送
void chassis_cmd_aotu(CAN_HandleTypeDef *Target_hcan)
{
    uint32_t send_mail_box;

    uint8_t TxData[8];
    CAN_DMstart_TxHeader.StdId = AUTO_MODE_CMD;
    CAN_DMstart_TxHeader.IDE = CAN_ID_STD;
    CAN_DMstart_TxHeader.RTR = CAN_RTR_DATA;
    CAN_DMstart_TxHeader.DLC = 0x08;

    Float_to_Byte(ball_track_target.speed, ball_track_target.angle, TxData);

    HAL_CAN_AddTxMessage(Target_hcan, &CAN_DMstart_TxHeader, TxData, &send_mail_box);
}

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;

/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float a, float b, unsigned char byte[])
{
    FloatLongType fl, f2;
    fl.fdata = a;
    f2.fdata = b;
    byte[0] = (unsigned char)fl.ldata;
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
    byte[4] = (unsigned char)f2.ldata;
    byte[5] = (unsigned char)(f2.ldata >> 8);
    byte[6] = (unsigned char)(f2.ldata >> 16);
    byte[7] = (unsigned char)(f2.ldata >> 24);
}
/*
将4个字节数据byte[4]转化为浮点数存放在*f中
*/
// 用于接收上位机的三个float数据，
void Byte_to_Float(float *fx, float *fy, float *fz, unsigned char byte[])
{
    FloatLongType flx, fly, flz;
    flx.ldata = 0;
    fly.ldata = 0;
    flz.ldata = 0;

    flz.ldata = byte[11];
    flz.ldata = (flz.ldata << 8) | byte[10];
    flz.ldata = (flz.ldata << 8) | byte[9];
    flz.ldata = (flz.ldata << 8) | byte[8];
    fly.ldata = byte[7];
    fly.ldata = (fly.ldata << 8) | byte[6];
    fly.ldata = (fly.ldata << 8) | byte[5];
    fly.ldata = (fly.ldata << 8) | byte[4];
    flx.ldata = byte[3];
    flx.ldata = (flx.ldata << 8) | byte[2];
    flx.ldata = (flx.ldata << 8) | byte[1];
    flx.ldata = (flx.ldata << 8) | byte[0];

    *fx = flx.fdata;
    *fy = fly.fdata;
    *fz = flz.fdata;
}

void MD_motor_SendCurrent(CAN_HandleTypeDef *hcan, uint32_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint32_t send_mail_box;
    uint8_t txData[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp; // 声明临时变量

    _pos = fminf(fmaxf(P_MIN, _pos), P_MAX);
    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    CAN_DMmsg_TxHeader.StdId = id;
    CAN_DMmsg_TxHeader.IDE = CAN_ID_STD;
    CAN_DMmsg_TxHeader.RTR = CAN_RTR_DATA;
    CAN_DMmsg_TxHeader.DLC = 0x08;
    txData[0] = pos_tmp >> 8;
    txData[1] = pos_tmp & 0xff;
    txData[2] = vel_tmp >> 4;
    txData[3] = ((vel_tmp & 0xf) << 4) | (kp_tmp >> 8);
    txData[4] = kp_tmp & 0xff;
    txData[5] = kd_tmp >> 4;
    txData[6] = ((kd_tmp & 0xf) << 4) | (tor_tmp >> 8);
    txData[7] = tor_tmp & 0xff;

    HAL_CAN_AddTxMessage(hcan, &CAN_DMmsg_TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header_can1, rx_header_can2;
    uint8_t rx_data_can1[8];
    uint8_t rx_data_can2[8];
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header_can1, rx_data_can1);

        switch (rx_header_can1.StdId)
        {
        case CAN_3508_M5_ID:
        {
            static uint8_t i = 0;
            i = rx_header_can1.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_Date[i], rx_data_can1);

            break;
        }
        case DT7_RX_ch:
        {
            DBUS_ReceiveData.ch0 = rx_data_can1[0] << 8 | rx_data_can1[1];
            DBUS_ReceiveData.ch1 = rx_data_can1[2] << 8 | rx_data_can1[3];
            DBUS_ReceiveData.ch2 = rx_data_can1[4] << 8 | rx_data_can1[5];
            DBUS_ReceiveData.ch3 = rx_data_can1[6] << 8 | rx_data_can1[7];
            break;
        }

        case DT7_RX_S:
        {
            DBUS_ReceiveData.dial = rx_data_can1[0] << 8 | rx_data_can1[1];
            DBUS_ReceiveData.switch_left = rx_data_can1[2];
            DBUS_ReceiveData.switch_right = rx_data_can1[3];
            break;
        }

        default:
        {
            break;
        }
        }
    }

    if (hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header_can2, rx_data_can2); // 从FIFO中接收消息至rx_header_can2
        switch (rx_header_can2.StdId)
        {

        case DM4340_M1:
        {
            DM_circle_num[0]++;
            DM4340_Date[0].id = (rx_data_can2[0]) & 0x0F;
            MD_CanReceive(&DM4340_Date[0], rx_data_can2);

            DM4340_Date[0].esc_back_position_last = DM4340_Date[0].esc_back_position;
            DM4340_Date[0].real_angle = DM4340_Date[0].esc_back_position / PI * 180; // RUD_DirAngle_Proc(DM4340_Date[0].serial_angle);

            break;
        }
        case DM4340_M2:
        {
            DM_circle_num[1]++;
            DM4340_Date[1].id = (rx_data_can2[0]) & 0x0F;
            MD_CanReceive(&DM4340_Date[1], rx_data_can2);

            DM4340_Date[1].esc_back_position_last = DM4340_Date[1].esc_back_position;
            DM4340_Date[1].real_angle = DM4340_Date[1].esc_back_position / PI * 180;
            break;
        }
        case DM4340_M3:
        {
            DM_circle_num[2]++;
            DM4340_Date[2].id = (rx_data_can2[0]) & 0x0F;
            MD_CanReceive(&DM4340_Date[2], rx_data_can2);

            DM4340_Date[2].esc_back_position_last = DM4340_Date[2].esc_back_position;
            DM4340_Date[2].real_angle = DM4340_Date[2].esc_back_position / PI * 180;
            ;
            break;
        }
        }
    }
}

// 达妙电机的数据解包和赋值
void MD_CanReceive(DM4340_motor_data_t *motor, uint8_t RxDate[8])
{

    int p_int = (RxDate[1] << 8) | RxDate[2];
    int v_int = (RxDate[3] << 4) | (RxDate[4] >> 4);
    int i_int = ((RxDate[4] & 0xf) << 8) | (RxDate[5]);
    int T_int = RxDate[6];
    if (motor->id == 0x01)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, P_MIN, P_MAX, 16); // 电机位置
        // motor->esc_back_speed = uint_to_float(v_int,V_MIN,V_MAX,12)*100; // 电机速度
        motor->esc_back_current = uint_to_float(i_int, T_MIN, T_MAX, 12); //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
    if (motor->id == 0x02)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, P_MIN, P_MAX, 16); // 电机位置
        // motor->esc_back_speed = uint_to_float(v_int,V_MIN,V_MAX,12)*100; // 电机速度
        motor->esc_back_current = uint_to_float(i_int, T_MIN, T_MAX, 12); //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
    if (motor->id == 0x03)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, P_MIN, P_MAX, 16); // 电机位置
                                                                           // motor->esc_back_speed = uint_to_float(v_int,V_MIN,V_MAX,12)*100; // 电机速度
        motor->esc_back_current = uint_to_float(i_int, T_MIN, T_MAX, 12);  //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
    if (motor->id == 0x04)
    {
        motor->state = (RxDate[0]) >> 4;
        motor->esc_back_position = uint_to_float(p_int, P_MIN, P_MAX, 16); // 电机位置
        // motor->esc_back_speed = uint_to_float(v_int,V_MIN,V_MAX,12)*100; // 电机速度
        motor->esc_back_current = uint_to_float(i_int, T_MIN, T_MAX, 12); //	电机扭矩/电流
        motor->Tmos = (float)(RxDate[6]);
        motor->Tcoil = (float)(RxDate[7]);
    }
}

// 过零检测
float RUD_DirAngle_Proc(float Angle)
{
    while (Angle > 360 || Angle < 0)
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

// 浮点数转整形
int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min)
        x = x_min;
    else if (x > x_max)
        x = x_max;

    return (int)((x - x_min) * ((float)((1 << bits) / span)));
}
// 整形转浮点数
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

const motor_measure_t *get_3508_M5_motor_measure_point(void)
{
    return &motor_Date[4];
}
const DM4340_motor_data_t *get_4340_M1_motor_measure_point(void)
{
    return &DM4340_Date[0];
}
const DM4340_motor_data_t *get_4340_M2_motor_measure_point(void)
{
    return &DM4340_Date[1];
}
const DM4340_motor_data_t *get_4340_M3_motor_measure_point(void)
{
    return &DM4340_Date[2];
}
