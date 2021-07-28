#include "bsp_usb.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>


static uint8_t usb_buf[512];


/**
  * @brief          通过usb虚拟串口打印调试信息，串口调试软件推荐Microsoft Store里的串口调试助手
  * @param[in]      类似printf
  * @retval         none
  */
void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    usb_transmit(usb_buf, len);
}

/**
  * @brief          通过usb虚拟串口发送数据
  * @param[in]      data: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
void usb_transmit(uint8_t *data, uint16_t len)
{
    CDC_Transmit_FS(data, len);
}
