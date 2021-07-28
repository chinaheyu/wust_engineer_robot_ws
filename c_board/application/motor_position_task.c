/**
  ******************************************************************************
  * @file       motor_position_task.c/h
  * @brief      ���λ�û���������
  * @author     heyu@wust.edu.cn
  @verbatim
  ==============================================================================
  1. ��Ҫʹ��λ�û��ĵ�����յ������й���ÿ���ĸ���������IDΪ1234����5678
  2. �������position_init�����н����������ʼ��
  3. ������İ�2 ���������7����������ID��Ӧ1234567
  4. ���̵��ʹ�õ���can1��1234��������������λ�û�
  5. ͨ��set_motor_position���õ��λ�ã�get_motor_position��ȡ��ǰλ��
  6. ʹ��reset_all_motor_ID�����õ�������������IDģʽ
  ==============================================================================
  @endverbatim
  ******************************************************************************
  */
#include "motor_position_task.h"
#include "cmsis_os.h"
#include "string.h"
#include "bsp_usb.h"
#include "math.h"
#include "remote_control.h"
#include "bsp_buzzer.h"

int is_cali_pose;

/**
  * @brief          ��ʼ�������λ�û����ڴ������Ҫλ�û��ĵ��������
  * @retval         none
  */
static void position_init(void);

/**
  * @brief          ���·�������
  * @param[in]      motor: �����ָ��
  * @retval         none
  */
static void position_feedback_update(Motor* motor);

/**
  * @brief          ���������ٶȻ���λ�û�
  * @param[in]      motor_group: �����ָ��
  * @retval         none
  */
static void position_control_loop(Motor* motor);

/**
  * @brief          ��ʼ������ṹ��
  * @param[in]      motor_group: ������ָ��
  * @param[in]      motor_index: ����ڵ�����е����(0-3)����Ӧ���1234����5678
  * @param[in]      ecd_range: ������Ա����������Χ
  * @retval         none
  */
static void motor_init(MotorGroup* motor_group, int motor_index, int ecd_range);

/**
  * @brief          ��ʼ�������ṹ��
  * @param[in]      motor_group: ������ָ��
  * @param[in]      hcan_ptr: ������Ӧ��canָ��
  * @param[in]      motor_id: ��ʶ������־�����ĵ��ID��1234����4567
  * @retval         none
  */
static void motor_group_init(MotorGroup* motor_group, CAN_HandleTypeDef *hcan_ptr, uint32_t motor_id);

/**
  * @brief          ͨ��USB���⴮���봮�ڵ������ֻ��Ƶ��λ������(���Microsoft Store��Ĵ��ڵ�������ʹ��)
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
static void plot_motor_position(Motor* motor);

/**
  * @brief          �첽�ƶ��������
  * @param[in]      argument: ����
  * @retval         none
  */
void asyn_move_to_position_by_acceleration_task(void const * argument);


osThreadDef(MOVE_MOTOR_THREAD, asyn_move_to_position_by_acceleration_task, osPriorityNormal, 16, 128);


#if INCLUDE_uxTaskGetStackHighWaterMark
UBaseType_t uxHighWaterMark;
#endif



//�������飬����Ҫ���Ƶĵ����Ҫ�Ž���Ӧ�ĵ����
static MotorGroup motor_groups[MOTOR_GROUP_COUNT];


/**
  * @brief          ��ʼ�������λ�û����ڴ������Ҫλ�û��ĵ��������
  * @retval         none
  */
static void position_init(void)
{
    //��ʼ���ڴ�����
    memset(motor_groups, 0, MOTOR_GROUP_COUNT * sizeof(MotorGroup));
    
    //���õ����
    motor_group_init(&motor_groups[0], &hcan2, MOTOR_ID_1234);
    motor_group_init(&motor_groups[1], &hcan2, MOTOR_ID_5678);
    motor_group_init(&motor_groups[2], &hcan1, MOTOR_ID_5678);
    
    //��ʼ�����
    motor_init(&motor_groups[0], 0, 8192);
    motor_init(&motor_groups[0], 1, 8192);
    motor_init(&motor_groups[0], 2, 8192);
    motor_init(&motor_groups[0], 3, 8192);
    
    motor_init(&motor_groups[1], 0, 8192);
    motor_init(&motor_groups[1], 1, 8192);
    motor_init(&motor_groups[1], 2, 8192);
    motor_init(&motor_groups[1], 3, 8192);
    
    motor_init(&motor_groups[2], 0, 8192);
    motor_init(&motor_groups[2], 1, 8192);
    motor_init(&motor_groups[2], 2, 8192);
    motor_init(&motor_groups[2], 3, 8192);
    
}

/**
  * @brief          ���õ����λ��
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @param[in]      position: �����λ��
  * @retval         none
  */
void set_motor_position(int group, int index, int position)
{
    motor_groups[group].motor[index].target_position = position;
}

/**
  * @brief          ��ȡ�����ǰ��λ��
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @retval         �����ǰ��λ��
  */
int get_motor_position(int group, int index)
{
    return motor_groups[group].motor[index].current_position;
}

/**
  * @brief          �����е�������������IDģʽ(�Ѱ󶨵�C���ϵİ���KEY,����KEY���ɿ������õ��ID)
  * @retval         none
  */
void reset_all_motor_ID(void)
{
    int i;
    for (i = 0; i < MOTOR_GROUP_COUNT; ++i)
    {
        //������ʹ�ܵĵ����
        if (motor_groups[i].enable_group)
        {
            cmd_group_reset_ID(&motor_groups[i]);
        }
    }
}

/**
  * @brief          ��ʼ������ṹ��
  * @param[in]      motor_group: ������ָ��
  * @param[in]      motor_index: ����ڵ�����е����(0-3)����Ӧ���1234����5678
  * @param[in]      ecd_range: ������Ա����������Χ
  * @retval         none
  */
static void motor_init(MotorGroup* motor_group, int motor_index, int ecd_range)
{
    motor_group->motor[motor_index].enable_motor = 1;
    motor_group->motor[motor_index].ecd_range = ecd_range;
    if (motor_group->motor_id == MOTOR_ID_1234)
    {
        motor_group->motor[motor_index].motor_measure = get_motor_measure_point(motor_group, motor_index + 1);
    }
    else
    {
        motor_group->motor[motor_index].motor_measure = get_motor_measure_point(motor_group, motor_index + 5);
    }
    motor_group->motor[motor_index].offset_ecd = motor_group->motor[motor_index].motor_measure->ecd;
    position_feedback_update(&motor_group->motor[motor_index]);
    PID_init(&motor_group->motor[motor_index].speed_pid, PID_POSITION, speed_pid_params, 16000.0f, 2000.0f);
    PID_init(&motor_group->motor[motor_index].position_pid, PID_POSITION, position_pid_params, 4000.0f, 800.0f);
}

/**
  * @brief          ��ʼ�������ṹ��
  * @param[in]      motor_group: ������ָ��
  * @param[in]      hcan_ptr: ������Ӧ��canָ��
  * @param[in]      motor_id: ��ʶ������־�����ĵ��ID��1234����4567
  * @retval         none
  */
static void motor_group_init(MotorGroup* motor_group, CAN_HandleTypeDef *hcan_ptr, uint32_t motor_id)
{
    motor_group->enable_group = 1;
    motor_group->hcan_ptr = hcan_ptr;
    motor_group->motor_id = motor_id;
}

/**
  * @brief          ���·�������
  * @param[in]      motor: �����ָ��
  * @retval         none
  */
static void position_feedback_update(Motor* motor)
{
    //��ȡ�����������
    motor->ecd = motor->motor_measure->ecd;
    motor->speed = motor->motor_measure->speed_rpm;
    
    //���µ��λ��
    int relative_ecd = motor->ecd - motor->offset_ecd;
    const int half_ecd_range = motor->ecd_range / 2;
    if (relative_ecd > half_ecd_range)
    {
        relative_ecd -= motor->ecd_range;
    }
    else if (relative_ecd < -half_ecd_range)
    {
        relative_ecd += motor->ecd_range;
    }
    motor->current_position += relative_ecd;
    motor->offset_ecd = motor->ecd;
}

/**
  * @brief          ���������ٶȻ���λ�û�
  * @param[in]      motor_group: �����ָ��
  * @retval         none
  */
static void position_control_loop(Motor* motor)
{
    motor->target_speed = PID_calc(&motor->position_pid, motor->current_position, motor->target_position);
    motor->current = PID_calc(&motor->speed_pid, motor->speed, motor->target_speed);
}

/**
  * @brief          ͨ��USB���⴮���봮�ڵ������ֻ��Ƶ��λ������(���Microsoft Store��Ĵ��ڵ�������ʹ��)
  * @param[in]      motor: ���ָ��
  * @retval         none
  */
static void plot_motor_position(Motor* motor)
{
    usb_printf("p=%d\r\n", motor->current_position);
}

/**
  * @brief          motor_position�������ڼ������Ĵ���PID���Ե������λ�ÿ���
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void motor_position_task(void const * argument)
{
    //wait a time 
    //����һ��ʱ��
    osDelay(400);
    
    //��ʼ��
    position_init();
    
    const RC_ctrl_t *controller = get_remote_control_point();
    
    int i, j;
    
    while (1)
    {
        
        //ң������Ƭ����OFF��������ʱֹͣ�����λ�û������ڴ���������
        if ((!switch_is_up(controller->rc.s[1])) && (!is_cali_pose))
        {
            for (i = 0; i < MOTOR_GROUP_COUNT; ++i)
            {
                //������ʹ�ܵĵ����
                if (motor_groups[i].enable_group)
                {
                    for (j = 0; j < 4; ++j)
                    {
                        if (motor_groups[i].motor[j].enable_motor)
                        {
                            //���ʹ�ܵ�������µ�����������㴮��PID
                            position_feedback_update(&motor_groups[i].motor[j]);
                            position_control_loop(&motor_groups[i].motor[j]);
                        }
                        else
                        {
                            //�����ʹ�ܵ����������0
                            motor_groups[i].motor[j].current = 0;
                        }
                    }
                    cmd_motor_group(&motor_groups[i]);
                }
            }
        }
        else
        {
            for (i = 0; i < MOTOR_GROUP_COUNT; ++i)
            {
                //������ʹ�ܵĵ����
                if (motor_groups[i].enable_group)
                {
                    for (j = 0; j < 4; ++j)
                    {
                        if (motor_groups[i].motor[j].enable_motor)
                        {
                            //���ʹ�ܵ�������µ������
                            position_feedback_update(&motor_groups[i].motor[j]);
                        }
                        motor_groups[i].motor[j].current = 0;
                    }
                    cmd_motor_group(&motor_groups[i]);
                }
            }
        }
        
        //�������λ������ͼ�����ڵ�PID
        //plot_motor_position(&motor_groups[0].motor[0]);
        
        osDelay(2);
        
#if INCLUDE_uxTaskGetStackHighWaterMark
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          �ѵ����ǰ��λ������Ϊ���λ�õ�ԭ��
  * @param[in]      motor: �����ָ��
  * @retval         none
  */
void set_motor_zero_position(int group, int index)
{
    motor_groups[group].motor[index].current_position = 0;
    motor_groups[group].motor[index].target_position = 0;
}

/**
  * @brief          ʹ�ܵ����λ�û�
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @retval         none
  */
void enable_motor_position_control(int group, int index)
{
    motor_groups[group].motor[index].enable_motor = 1;
}

/**
  * @brief          ʧ�ܵ����λ�û�
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @retval         none
  */
void disable_motor_position_control(int group, int index)
{
    motor_groups[group].motor[index].enable_motor = 0;
}

/**
  * @brief          ͨ���ȼ����˶�ƽ���ƶ����(����)
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @param[in]      accel: ���ٹ��̵ļ��ٶ�     (��λ: Ȧÿ�뷽)
  * @param[in]      decel: ���ٹ��̵ļ��ٶ�     (��λ: Ȧÿ�뷽)
  * @param[in]      max_speed: ��������ٶ�   (��λ: Ȧÿ��)
  * @param[in]      position: ���Ŀ��λ��      (��λ: ��������ֵ)
  * @retval         none
  */
void move_to_position_by_acceleration(int group, int index, float accel, float decel, float max_speed, int position)
{
    accel *= motor_groups[group].motor[index].ecd_range;
    decel *= motor_groups[group].motor[index].ecd_range;
    max_speed *= motor_groups[group].motor[index].ecd_range;
    float t, t0, t1, t2, t3, d0, d1, d2;
    const float dmax = 0.5f * max_speed * max_speed / accel + 0.5f * max_speed * max_speed / decel;
    const float d = position - motor_groups[group].motor[index].current_position;
    
    d0 = motor_groups[group].motor[index].current_position;
    t0 = (float)osKernelSysTick() / (float)osKernelSysTickFrequency;
    
    if(d > 0)
    {
        decel = -decel;
    }
    else
    {
        accel = -accel;
        max_speed = -max_speed;
    }
    
    if (dmax < fabs(d))
    {
        t1 = max_speed / accel + t0;
        t2 = (fabs(d) - dmax) / fabs(max_speed) + t1;
        t3 = -max_speed / decel + t2;
        d1 = 0.5f * accel * (t1 - t0) * (t1 - t0) + d0;
        d2 = max_speed * (t2 - t1) + d1;
        
        while (1)
        {
            t = (float)osKernelSysTick() / (float)osKernelSysTickFrequency;
            
            if (t < t1)
            {
                motor_groups[group].motor[index].target_position = d0 + 0.5f * accel * (t - t0) * (t - t0);
            }
            else if (t < t2)
            {
                motor_groups[group].motor[index].target_position = d1 + max_speed * (t - t1);
            }
            else if (t < t3)
            {
                motor_groups[group].motor[index].target_position = d2 + max_speed * (t - t2) + 0.5f * decel * (t - t2) * (t - t2);
            }
            else
            {
                motor_groups[group].motor[index].target_position = position;
                break;
            }
            osDelay(10);
        }
    }
    else
    {
        const float vmax = d / fabs(d) * sqrt(-2.0f * accel * decel * d / (accel - decel));
        
        t1 = vmax / accel + t0;
        t2 = -vmax / decel + t1;
        
        d1 = 0.5f * accel * (t1 - t0) * (t1 - t0) + d0;
       
        while (1)
        {
            t = (float)osKernelSysTick() / (float)osKernelSysTickFrequency;
            
            if (t < t1)
            {
                motor_groups[group].motor[index].target_position = d0 + 0.5f * accel * (t - t0) * (t - t0);
            }
            else if (t < t2)
            {
                motor_groups[group].motor[index].target_position = d1 + vmax * (t - t1) + 0.5f * decel * (t - t1) * (t - t1);
            }
            else
            {
                motor_groups[group].motor[index].target_position = position;
                break;
            }
            osDelay(10);
        }
    }
    
}

/**
  * @brief          �첽�ƶ��������
  * @param[in]      argument: ����
  * @retval         none
  */
void asyn_move_to_position_by_acceleration_task(void const * argument)
{
    move_args *args = (move_args*)argument;
    
    move_to_position_by_acceleration(args->group, args->index, args->accel, args->decel, args->max_speed, args->position);
    
    args->is_finished = 1;
    
    osThreadTerminate(osThreadGetId());
}

/**
  * @brief          �첽(������)ͨ���ȼ����˶�ƽ���ƶ����
  * @param[in]      move_args: ����˶�������ָ��
  * @retval         none
  */
void asyn_move_to_position_by_acceleration(move_args* args)
{
    args->is_finished = 0;
    osThreadCreate(osThread(MOVE_MOTOR_THREAD), args);
}

/**
  * @brief          �������ȴ�����˶�����
  * @param[in]      move_args: ����˶�������ָ��
  * @retval         none
  */
void wait_motor_move_to_position(move_args* args)
{
    while (!args->is_finished)
    {
        osDelay(10);
    }
}

void cali_pose_hock(void)
{
    is_cali_pose = 1;
    buzzer_on(31, 19999);
    for (int k = 0; k < 10; ++k)
    {
        for (int i = 0; i < MOTOR_GROUP_COUNT; ++i)
        {
            //������ʹ�ܵĵ����
            if (motor_groups[i].enable_group)
            {
                for (int j = 0; j < 4; ++j)
                {
                    if (motor_groups[i].motor[j].enable_motor)
                    {
                        //���ʹ�ܵ�������µ������
                        motor_groups[i].motor[j].current_position = 0;
                        motor_groups[i].motor[j].target_position = 0;
                        
                    }
                }
            }
        }
        osDelay(1000);
    }
    is_cali_pose = 0;
}
