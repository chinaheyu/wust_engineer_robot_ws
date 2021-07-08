#ifndef WUST_BASE_H
#define WUST_BASE_H

#include <stdint.h>

#define USB_FRAME_HEADER                 0xA5
#define USB_FRAME_HEADER_SIZE            sizeof(frame_header_t)
#define USB_FRAME_CMD_SIZE               2
#define USB_FRAME_CRC16_SIZE             2
#define USB_FRAME_HEADER_CRC_LEN                  (USB_FRAME_HEADER_SIZE + USB_FRAME_CRC16_SIZE)
#define USB_FRAME_HEADER_CRC_CMDID_LEN            (USB_FRAME_HEADER_SIZE + USB_FRAME_CRC16_SIZE + sizeof(uint16_t))
#define USB_FRAME_HEADER_CMDID_LEN                (USB_FRAME_HEADER_SIZE + sizeof(uint16_t))

#define USB_FRAME_MAX_SIZE         128

typedef struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
}__attribute__((packed)) frame_header_t;

typedef struct
{
    float vx;
    float vy;
    float vw;
}__attribute__((packed)) cmd_vel_t;



#endif