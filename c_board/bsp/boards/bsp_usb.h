#ifndef BSP_USB_H
#define BSP_USB_H

#include "struct_typedef.h"

extern void usb_printf(const char *fmt,...);
extern void usb_transmit(uint8_t *data, uint16_t len);

#endif
