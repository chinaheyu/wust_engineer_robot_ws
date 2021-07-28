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


typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct
{
    double position_x;
    double position_y;
    double angle_w;
}__attribute__((packed)) robot_position_t;

typedef struct
{
    frame_header_t *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[USB_FRAME_MAX_SIZE];
    unpack_step_e  unpack_step;
    uint16_t       index;
} unpack_data_t;

#endif