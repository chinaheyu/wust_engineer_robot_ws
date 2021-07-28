#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "wust_base/crc.h"
#include "wust_base/serial_device.h"
#include "wust_base/wust_base.h"


SerialDevice serial("/dev/serial_sdk");

cmd_vel_t cmd_vel;

uint8_t txBuf[USB_FRAME_MAX_SIZE];
uint8_t seq = 0;


void protocol_transmit(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
    /* Set frame header */
    auto *pHeader = (frame_header_t *)txBuf;
    pHeader->sof = 0xA5;
    pHeader->data_length = len;
    pHeader->seq = seq++;

    /* Calculate data size */
    uint16_t headSize = USB_FRAME_HEADER_SIZE;
    uint16_t frameSize = len + USB_FRAME_HEADER_CRC_CMDID_LEN;

    /* Apend CRC */
    memcpy(txBuf + headSize, &cmd_id, sizeof(cmd_id));
    append_crc8(txBuf, headSize);
    memcpy(txBuf + headSize + sizeof(cmd_id), data, len);
    append_crc16(txBuf, frameSize);

    serial.write(txBuf, frameSize);
}

void velCallback(const geometry_msgs::Twist::ConstPtr& data)
{
    cmd_vel.vx = data->linear.x;
    cmd_vel.vy = data->linear.y;
    cmd_vel.vw = data->angular.z;
    protocol_transmit(0x0101, (uint8_t*)&cmd_vel, sizeof(cmd_vel_t));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wust_base");
    ros::NodeHandle nh;
    
    ros::Subscriber vel_sub = nh.subscribe("/cmd_vel", 10, velCallback);

    ros::spin();
    
    return 0;
}
