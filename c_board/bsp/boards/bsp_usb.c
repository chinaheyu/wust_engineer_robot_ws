#include "bsp_usb.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdarg.h>


static uint8_t usb_buf[512];


/**
  * @brief          ͨ��usb���⴮�ڴ�ӡ������Ϣ�����ڵ�������Ƽ�Microsoft Store��Ĵ��ڵ�������
  * @param[in]      ����printf
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
  * @brief          ͨ��usb���⴮�ڷ�������
  * @param[in]      data: ����ָ��
  * @param[in]      len: ���ݳ���
  * @retval         none
  */
void usb_transmit(uint8_t *data, uint16_t len)
{
    CDC_Transmit_FS(data, len);
}
