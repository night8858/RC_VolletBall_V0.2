#include "date_exchange.h"
#include "main.h"
#include "top_ctrl.h"
#include "ball_track.h"
#include "bsp_usart.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

extern DBUSDecoding_Type DBUS_ReceiveData;    // 底盘发来的dbus的数据
extern ball_track_target_t ball_track_target; // 球追踪目标点
extern Ball_Pos RX_ball_pos;

void CMD_to_chassis_task(void const * argument)
{
    //初始化数值设定为没有排球的情况，防止启动乱跑
    RX_ball_pos.ball_pos_x = 320;
    RX_ball_pos.ball_pos_y = 240;
    RX_ball_pos.ball_pos_z = 0;
    //RX_ball_pos.White_ratio = 0;
	
    while(1)
    {
		//uart_dma_printf(&huart1, "%4.3f ,%4.3f ,%4.3f\n",RX_ball_pos.ball_pos_x, RX_ball_pos.ball_pos_y, RX_ball_pos.ball_pos_z );
        osDelay(1);
        if(DBUS_ReceiveData.switch_left == 3 && DBUS_ReceiveData.switch_right == 3)
        {
            if (ball_track_target.mode_falg == 0)
            {
                ball_track_target.speed = 0;
                ball_track_target.angle = 0;
            }
            
            chassis_cmd_aotu(&hcan1);
            osDelay(2);
        } 
    }
}
