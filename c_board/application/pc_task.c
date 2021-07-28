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
#include "pc_task.h"
#include "cmsis_os.h"
#include "fifo.h"
#include "CRC8_CRC16.h"
#include "bsp_usb.h"
#include "detect_task.h"


#define USB_FIFO_BUF_LENGTH 1024

static int seq = 0;
static uint8_t txBuf[USB_FRAME_MAX_SIZE];

fifo_s_t usb_fifo;
uint8_t usb_fifo_buf[USB_FIFO_BUF_LENGTH];
usb_unpack_data_t unpack_obj;

static void unpack_fifo_data(void);
static void usb_data_solve(uint8_t *frame);
static void usb_data_handle(uint16_t cmd_id, uint8_t *data);

static pc_cmd_vel_t pc_cmd_vel;

void pc_task(void const * argument)
{
    //��ʼ��FIFO������
    fifo_s_init(&usb_fifo, usb_fifo_buf, USB_FIFO_BUF_LENGTH);
    
    const error_t *error_list = get_error_list_point();
    
    
    while (1)
    {
        unpack_fifo_data();
        
        osDelay(10);
    }
}

/**
  * @brief          ���ֽڽ��
  * @param[in]      void
  * @retval         none
  */
static void unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = USB_FRAME_HEADER;
  usb_unpack_data_t *p_obj = &unpack_obj;

  while ( fifo_s_used(&usb_fifo) )
  {
    byte = fifo_s_get(&usb_fifo);
    switch(p_obj->unpack_step)
    {
      case USB_STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = USB_STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case USB_STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = USB_STEP_LENGTH_HIGH;
      }break;
      
      case USB_STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (USB_FRAME_MAX_SIZE - USB_FRAME_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = USB_STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = USB_STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case USB_STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = USB_STEP_HEADER_CRC8;
      }break;

      case USB_STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == USB_FRAME_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, USB_FRAME_HEADER_SIZE) )
          {
            p_obj->unpack_step = USB_STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = USB_STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case USB_STEP_DATA_CRC16:
      {
        if (p_obj->index < (USB_FRAME_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (USB_FRAME_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = USB_STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, USB_FRAME_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            usb_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = USB_STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

/**
  * @brief          ��������������֡
  * @param[in]      frame: ����ָ֡��
  * @retval         none
  */
static void usb_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    static usb_frame_header_t receive_header;
    memcpy(&receive_header, frame, sizeof(usb_frame_header_t));

    index += sizeof(usb_frame_header_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    
    usb_data_handle(cmd_id, frame + index);

}

/**
  * @brief          �ڴ˴����յ���ָ�������
  * @param[in]      frame: ����ָ֡��
  * @retval         none
  */
static void usb_data_handle(uint16_t cmd_id, uint8_t *data)
{
    switch (cmd_id)
    {
        case 0x0001:
        {
            //ͨ��PC������������֡(����Ϊ0��cmd_idΪ0x0001)�����PC���Ƿ�����
            detect_hook(PC_TOE);
            break;
        }
        case 0x0101:
        {
            memcpy(&pc_cmd_vel, data, sizeof(pc_cmd_vel_t));
            break;
        }
        default:
        {
            break;
        }
    }
    
}

/**
  * @brief          ͨ��Э�鷢��ָ�������
  * @param[in]      cmd_id: ָ��
  * @param[in]      data: ����ָ��
  * @param[in]      len: ���ݳ���
  * @retval         none
  */
void usb_send_data(uint16_t cmd_id, uint8_t *data, uint16_t len)
{
    //���֡ͷ
    usb_frame_header_t *pHeader = (usb_frame_header_t*)txBuf;
    pHeader->sof = USB_FRAME_HEADER;
    pHeader->data_length = len;
    pHeader->seq = seq++;
    
    //����֡��
    uint16_t headSize = USB_FRAME_HEADER_SIZE;
    uint16_t frameSize = len + USB_FRAME_HEADER_SIZE;
    
    //��ָ�������ݿ��������ͻ�����
    memcpy(txBuf + headSize, &cmd_id, sizeof(cmd_id));
    memcpy(txBuf + headSize + sizeof(cmd_id), data, len);
    
    //���CRCУ���
    append_CRC8_check_sum(txBuf, headSize);
    append_CRC16_check_sum(txBuf, frameSize);
    
    //��������
    usb_transmit(txBuf, frameSize);
}

/**
  * @brief          ��PC�Ͽ����ӵĻص�����
  * @retval         none
  */
void solve_lost_fun(void)
{
    memset(&pc_cmd_vel, 0, sizeof(pc_cmd_vel_t));
}

/**
  * @brief          ��ȡPC�ٶ�ָ�����ݵ�ָ��
  * @retval         pc_cmd_vel_t*
  */
const pc_cmd_vel_t *get_pc_cmd_vel_pointer(void)
{
    return &pc_cmd_vel;
}

