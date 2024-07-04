#include "main.h"
#include "DateExchange.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "remote_control.h"
#include "can_recv.h"

extern RC_ctrl_t rc_ctrl;

void DateExchange_Task(void const * argument)
{
    while (1)
    {
    osDelay(2);
    CAN_cmd_DT7_ch_Date();
    osDelay(2);
    CAN_cmd_DT7_s_Date(); 
    }
    

}

 