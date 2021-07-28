/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "bsp_usb.h"
#include <stdio.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"

#include "tim.h"


float get_time_ms_us(void);

static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
    error_list_usb_local = get_error_list_point();
    int debug_message_seq = 0;

    while(1)
    {
        osDelay(2000);
        
        //当PC机离线时通过USB串口打印调试信息，PC机在线时停止打印
        if (error_list_usb_local[PC_TOE].error_exist)
        {
            usb_printf(
"********** seq: %-3d **********\r\n\
voltage percentage:%d%%\r\n\
DBUS:%s\r\n\
chassis motor1:%s\r\n\
chassis motor2:%s\r\n\
chassis motor3:%s\r\n\
chassis motor4:%s\r\n\
gyro sensor:%s\r\n\
accel sensor:%s\r\n\
mag sensor:%s\r\n\
pc:%s\r\n\
referee usart:%s\r\n\
******************************\r\n",
            debug_message_seq,
            get_battery_percentage(), 
            status[error_list_usb_local[DBUS_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
            status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
            status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
            status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
            status[error_list_usb_local[PC_TOE].error_exist],
            status[error_list_usb_local[REFEREE_TOE].error_exist]);
        }
        
        debug_message_seq++;
        if (debug_message_seq > 999)
        {
            debug_message_seq = 0;
        }
    }

}

