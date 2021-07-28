#ifndef CRC_H
#define CRC_H

#include <stdint.h>

uint8_t get_crc8(uint8_t *p_msg, uint32_t len, uint8_t crc8);
uint32_t verify_crc8(uint8_t *p_msg, uint32_t len);
void append_crc8(uint8_t *p_msg, uint32_t len);
uint16_t get_crc16(uint8_t *p_msg, uint16_t len, uint16_t crc16);
uint32_t verify_crc16(uint8_t *p_msg, uint32_t len);
void append_crc16(uint8_t *p_msg, uint16_t len);



#endif