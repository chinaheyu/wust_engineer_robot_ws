#include "snail_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "bsp_fric.h"


void snail_task(void const * argument)
{
    const RC_ctrl_t *controller = get_remote_control_point();
    fric_off();
    while (1)
    {
        
        if (controller->rc.ch[4] > 10)
        {
            fric1_on(FRIC_OFF + 1.5 * controller->rc.ch[4]);
        }
        else
        {
            fric_off();
        }

        
        
        osDelay(10);
    }
}
