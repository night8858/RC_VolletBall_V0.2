#include "date_exchange.h"
#include "main.h"
#include "top_ctrl.h"
#include "ball_track.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

extern DBUSDecoding_Type DBUS_ReceiveData;    // 底盘发来的dbus的数据
extern ball_track_target_t ball_track_target; // 球追踪目标点

void CMD_to_chassis_task(void const * argument)
{
    while(1)
    {
        osDelay(10);
        if(DBUS_ReceiveData.switch_left == 3 && DBUS_ReceiveData.switch_right == 3)
        {
            chassis_cmd_aotu(&hcan1);
            osDelay(5);
        } 
    }
}
