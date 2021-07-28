#include "odometer_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include <math.h>

#define CHASSIS_MOTOR_ENCODER_RANGE 8192.0f
#define CHASSIS_MOTOR_ENCODER_RANGE_HALF 4096.0f
#define WHEEL_PERIMETER 478.0f
#define CHASSIS_MOTOR_DECELE_RATIO (187.0f / 3591.0f)
#define WHEELBASE 376
#define WHEELTRACK 400

/* gimbal is relative to chassis center x axis offset(mm) */
#define ROTATE_X_OFFSET 0
/* gimbal is relative to chassis center y axis offset(mm) */
#define ROTATE_Y_OFFSET 0


static const motor_measure_t *chassis_measure[4];
static float last_ecd[4];
static float total_ecd[4];

static robot_position_t robot_position;

static void odometer_init(void)
{
    for (int i = 0; i < 4; ++i)
    {
        chassis_measure[i] = get_chassis_motor_measure_point(i);
    }
}

static void update_chassis_motor_ecd(void)
{
    float relative_ecd, current_ecd;
    for (int i = 0; i < 4; ++i)
    {
        current_ecd = chassis_measure[i]->ecd;
        relative_ecd = current_ecd - last_ecd[i];
        if (relative_ecd > CHASSIS_MOTOR_ENCODER_RANGE_HALF)
        {
            relative_ecd -= CHASSIS_MOTOR_ENCODER_RANGE;
        }
        else if (relative_ecd < -CHASSIS_MOTOR_ENCODER_RANGE_HALF)
        {
            relative_ecd += CHASSIS_MOTOR_ENCODER_RANGE;
        }
        total_ecd[i] += relative_ecd;
        last_ecd[i] = current_ecd;
    }
}
    

void odometer_task(void const *argument)
{
    odometer_init();
    
    // 获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
    const fp32 *euler_angle = get_INS_angle_point();
    
    static double last_d_x, last_d_y, last_d_w, d_x, d_y, d_w, diff_d_x, diff_d_y, diff_d_w, mecanum_angle;
    
    const float ecd_ratio = WHEEL_PERIMETER * CHASSIS_MOTOR_DECELE_RATIO / (4 * CHASSIS_MOTOR_ENCODER_RANGE);
    const float rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET + ROTATE_Y_OFFSET);
    const float rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_X_OFFSET - ROTATE_Y_OFFSET);
    const float rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET - ROTATE_Y_OFFSET);
    const float rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_X_OFFSET + ROTATE_Y_OFFSET);
    
    
    while (1)
    {
        update_chassis_motor_ecd();
        
        last_d_x = d_x;
        last_d_y = d_y;
        last_d_w = d_w;
        
        d_x = ecd_ratio * (-total_ecd[0] + total_ecd[1] + total_ecd[2] - total_ecd[3]);
        d_y = ecd_ratio * (-total_ecd[0] - total_ecd[1] + total_ecd[2] + total_ecd[3]);
        d_w = ecd_ratio * (-total_ecd[0] / rotate_ratio_fr - total_ecd[1] / rotate_ratio_fl - total_ecd[2] / rotate_ratio_bl - total_ecd[3] / rotate_ratio_br);

        diff_d_x = d_x - last_d_x;
        diff_d_y = d_y - last_d_y;
        diff_d_w = d_w - last_d_w;

        mecanum_angle = euler_angle[0];
        
        robot_position.position_x += diff_d_x * cos(mecanum_angle) - diff_d_y * sin(mecanum_angle);
        robot_position.position_y += diff_d_x * sin(mecanum_angle) + diff_d_y * cos(mecanum_angle);
        robot_position.angle_w += diff_d_w;
        
        osDelay(2);
    }
}

const robot_position_t *get_robot_position_pointer(void)
{
    return &robot_position;
}

/**
  * @brief          移动相对距离
  * @param[in]      dx: 相对x轴移动的距离(mm)
  * @param[in]      dy: 相对y轴移动的距离(mm)
  * @param[in]      dw: 相对转动的角度(degree)
  * @retval         none
  */
void move_to_relative_position(float dx, float dy, float dw)
{
    // TODO: 暂时没想出来有啥用，先不做了
}
