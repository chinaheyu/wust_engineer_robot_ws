/**
  ******************************************************************************
  * @file       pc_task.c/h
  * @brief      ����λ����ͨѶ����ͨѶЭ�����Ʋ���ϵͳͨѶЭ��v1.2
  * @author     heyu@wust.edu.cn
  @verbatim
  ==============================================================================
  1. ���׵�PC������: https://github.com/chinaheyu/wust_engineer_robot_ws
  2. ͨ��usb_send_data��PC��������֡
  3. PC������������֡����н���������usb_data_handle�з��ദ��
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
#ifndef PC_TASK
#define PC_TASK

#include "stdint.h"


#define USB_FRAME_HEADER                 0xA5
#define USB_FRAME_HEADER_SIZE            sizeof(usb_frame_header_t)
#define USB_FRAME_CMD_SIZE               2
#define USB_FRAME_CRC16_SIZE             2
#define USB_FRAME_HEADER_CRC_LEN         (USB_FRAME_HEADER_SIZE + USB_FRAME_CRC16_SIZE)
#define USB_FRAME_HEADER_CRC_CMDID_LEN   (USB_FRAME_HEADER_SIZE + USB_FRAME_CRC16_SIZE + sizeof(uint16_t))
#define USB_FRAME_HEADER_CMDID_LEN       (USB_FRAME_HEADER_SIZE + sizeof(uint16_t))
#define USB_FRAME_MAX_SIZE               128

typedef enum
{
    USB_STEP_HEADER_SOF  = 0,
    USB_STEP_LENGTH_LOW  = 1,
    USB_STEP_LENGTH_HIGH = 2,
    USB_STEP_FRAME_SEQ   = 3,
    USB_STEP_HEADER_CRC8 = 4,
    USB_STEP_DATA_CRC16  = 5,
} usb_unpack_step_e;

typedef struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
}__attribute__((packed)) usb_frame_header_t;

typedef struct
{
    usb_frame_header_t *p_header;
    uint16_t       data_len;
    uint8_t        protocol_packet[USB_FRAME_MAX_SIZE];
    usb_unpack_step_e  unpack_step;
    uint16_t       index;
} usb_unpack_data_t;

// cmd_id: 0x0101
typedef struct
{
    float vx;
    float vy;
    float vw;
}__attribute__((packed)) pc_cmd_vel_t;

extern void pc_task(void const *pvParameters);

extern void usb_send_data(uint16_t cmd_id, uint8_t *data, uint16_t len);

/**
  * @brief          ��ȡPC�ٶ�ָ�����ݵ�ָ��
  * @retval         pc_cmd_vel_t*
  */
extern const pc_cmd_vel_t *get_pc_cmd_vel_pointer(void);

#endif

