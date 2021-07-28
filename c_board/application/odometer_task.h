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
  * @brief          �ƶ���Ծ���
  * @param[in]      dx: ���x���ƶ��ľ���(mm)
  * @param[in]      dy: ���y���ƶ��ľ���(mm)
  * @param[in]      dw: ���ת���ĽǶ�(degree)
  * @retval         none
  */
void move_to_relative_position(float dx, float dy, float dw);

extern void odometer_task(void const *pvParameters);


#endif

