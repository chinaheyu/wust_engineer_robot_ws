#ifndef ODOMETER_TASK_H
#define ODOMETER_TASK_H

typedef struct
{
    double position_x;
    double position_y;
    double angle_w;
}__attribute__((packed)) robot_position_t;


const robot_position_t *get_robot_position_pointer(void);

/**
  * @brief          移动相对距离
  * @param[in]      dx: 相对x轴移动的距离(mm)
  * @param[in]      dy: 相对y轴移动的距离(mm)
  * @param[in]      dw: 相对转动的角度(degree)
  * @retval         none
  */
void move_to_relative_position(float dx, float dy, float dw);

extern void odometer_task(void const *pvParameters);


#endif

