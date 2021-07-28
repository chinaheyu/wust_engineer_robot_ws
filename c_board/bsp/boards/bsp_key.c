#include "bsp_key.h"



int is_key_pressed(void)
{
    return HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET;
}


