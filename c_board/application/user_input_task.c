#include "user_input_task.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "bsp_usb.h"
#include "motor_position_task.h"
#include "bsp_key.h"



void on_Z_clicked(void)
{
    usb_printf("Z was clicked!\r\n");
}

void on_X_clicked(void)
{
    usb_printf("X was clicked!\r\n");
}

void on_C_clicked(void)
{
    usb_printf("C was clicked!\r\n");
}

void on_V_clicked(void)
{
    usb_printf("V was clicked!\r\n");
}

void on_B_clicked(void)
{
    usb_printf("B was clicked!\r\n");
}

void on_Key_clicked(void)
{
    reset_all_motor_ID();
    usb_printf("Key was clicked!\r\n");
}


/**
  * @brief          user_input任务，检测按键ZXCVB并调用各自的回调函数
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void user_input_task(void const * argument)
{
    const RC_ctrl_t *controller = get_remote_control_point();

    while(1)
    {
        if (controller->key.v == KEY_PRESSED_OFFSET_Z)
        {
            while (controller->key.v == KEY_PRESSED_OFFSET_Z)
            {
                osDelay(1);
            }
            on_Z_clicked();
        }
        
        if (controller->key.v == KEY_PRESSED_OFFSET_X)
        {
            while (controller->key.v == KEY_PRESSED_OFFSET_X)
            {
                osDelay(1);
            }
            on_X_clicked();
        }
        
        if (controller->key.v == KEY_PRESSED_OFFSET_C)
        {
            while (controller->key.v == KEY_PRESSED_OFFSET_C)
            {
                osDelay(1);
            }
            on_C_clicked();
        }
        
        if (controller->key.v == KEY_PRESSED_OFFSET_V)
        {
            while (controller->key.v == KEY_PRESSED_OFFSET_V)
            {
                osDelay(1);
            }
            on_V_clicked();
        }
        
        if (controller->key.v == KEY_PRESSED_OFFSET_B)
        {
            while (controller->key.v == KEY_PRESSED_OFFSET_B)
            {
                osDelay(1);
            }
            on_B_clicked();
        }
        
        if (is_key_pressed() == 1)
        {
            while (is_key_pressed() == 1)
            {
                osDelay(1);
            }
            on_Key_clicked();
        }
        
        osDelay(10);
    }

}

