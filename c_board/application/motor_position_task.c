/**
  ******************************************************************************
  * @file       motor_position_task.c/h
  * @brief      电机位置环控制任务
  * @author     heyu@wust.edu.cn
  @verbatim
  ==============================================================================
  1. 需要使用位置环的电机按照电机组进行管理，每组四个电机，电调ID为1234或者5678
  2. 电机组在position_init函数中进行配置与初始化
  3. 电调中心板2 最多能驱动7个电机，电调ID对应1234567
  4. 底盘电机使用的是can1的1234，不可以用来做位置环
  5. 通过set_motor_position设置电机位置，get_motor_position获取当前位置
  6. 使用reset_all_motor_ID可以让电机进入快速设置ID模式
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
  * @brief          初始化电机的位置环，在此添加需要位置环的电机组与电机
  * @retval         none
  */
static void position_init(void);

/**
  * @brief          更新反馈数据
  * @param[in]      motor: 电机的指针
  * @retval         none
  */
static void position_feedback_update(Motor* motor);

/**
  * @brief          计算电机的速度环和位置环
  * @param[in]      motor_group: 电机的指针
  * @retval         none
  */
static void position_control_loop(Motor* motor);

/**
  * @brief          初始化电机结构体
  * @param[in]      motor_group: 电机组的指针
  * @param[in]      motor_index: 电机在电机组中的序号(0-3)，对应电机1234或者5678
  * @param[in]      ecd_range: 电机绝对编码器的最大范围
  * @retval         none
  */
static void motor_init(MotorGroup* motor_group, int motor_index, int ecd_range);

/**
  * @brief          初始化电机组结构体
  * @param[in]      motor_group: 电机组的指针
  * @param[in]      hcan_ptr: 电机组对应的can指针
  * @param[in]      motor_id: 标识符，标志电机组的电机ID是1234还是4567
  * @retval         none
  */
static void motor_group_init(MotorGroup* motor_group, CAN_HandleTypeDef *hcan_ptr, uint32_t motor_id);

/**
  * @brief          通过USB虚拟串口与串口调试助手绘制电机位置曲线(配合Microsoft Store里的串口调试助手使用)
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
static void plot_motor_position(Motor* motor);

/**
  * @brief          异步移动电机任务
  * @param[in]      argument: 参数
  * @retval         none
  */
void asyn_move_to_position_by_acceleration_task(void const * argument);


osThreadDef(MOVE_MOTOR_THREAD, asyn_move_to_position_by_acceleration_task, osPriorityNormal, 16, 128);


#if INCLUDE_uxTaskGetStackHighWaterMark
UBaseType_t uxHighWaterMark;
#endif



//定义电机组，所有要控制的电机需要放进对应的电机组
static MotorGroup motor_groups[MOTOR_GROUP_COUNT];


/**
  * @brief          初始化电机的位置环，在此添加需要位置环的电机组与电机
  * @retval         none
  */
static void position_init(void)
{
    //初始化内存区域
    memset(motor_groups, 0, MOTOR_GROUP_COUNT * sizeof(MotorGroup));
    
    //配置电机组
    motor_group_init(&motor_groups[0], &hcan2, MOTOR_ID_1234);
    motor_group_init(&motor_groups[1], &hcan2, MOTOR_ID_5678);
    motor_group_init(&motor_groups[2], &hcan1, MOTOR_ID_5678);
    
    //初始化电机
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
  * @brief          设置电机的位置
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @param[in]      position: 电机的位置
  * @retval         none
  */
void set_motor_position(int group, int index, int position)
{
    motor_groups[group].motor[index].target_position = position;
}

/**
  * @brief          获取电机当前的位置
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @retval         电机当前的位置
  */
int get_motor_position(int group, int index)
{
    return motor_groups[group].motor[index].current_position;
}

/**
  * @brief          让所有电机进入快速设置ID模式(已绑定到C板上的按键KEY,按下KEY即可快速设置电机ID)
  * @retval         none
  */
void reset_all_motor_ID(void)
{
    int i;
    for (i = 0; i < MOTOR_GROUP_COUNT; ++i)
    {
        //跳过不使能的电机组
        if (motor_groups[i].enable_group)
        {
            cmd_group_reset_ID(&motor_groups[i]);
        }
    }
}

/**
  * @brief          初始化电机结构体
  * @param[in]      motor_group: 电机组的指针
  * @param[in]      motor_index: 电机在电机组中的序号(0-3)，对应电机1234或者5678
  * @param[in]      ecd_range: 电机绝对编码器的最大范围
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
  * @brief          初始化电机组结构体
  * @param[in]      motor_group: 电机组的指针
  * @param[in]      hcan_ptr: 电机组对应的can指针
  * @param[in]      motor_id: 标识符，标志电机组的电机ID是1234还是4567
  * @retval         none
  */
static void motor_group_init(MotorGroup* motor_group, CAN_HandleTypeDef *hcan_ptr, uint32_t motor_id)
{
    motor_group->enable_group = 1;
    motor_group->hcan_ptr = hcan_ptr;
    motor_group->motor_id = motor_id;
}

/**
  * @brief          更新反馈数据
  * @param[in]      motor: 电机的指针
  * @retval         none
  */
static void position_feedback_update(Motor* motor)
{
    //获取电机测量数据
    motor->ecd = motor->motor_measure->ecd;
    motor->speed = motor->motor_measure->speed_rpm;
    
    //更新电机位置
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
  * @brief          计算电机的速度环和位置环
  * @param[in]      motor_group: 电机的指针
  * @retval         none
  */
static void position_control_loop(Motor* motor)
{
    motor->target_speed = PID_calc(&motor->position_pid, motor->current_position, motor->target_position);
    motor->current = PID_calc(&motor->speed_pid, motor->speed, motor->target_speed);
}

/**
  * @brief          通过USB虚拟串口与串口调试助手绘制电机位置曲线(配合Microsoft Store里的串口调试助手使用)
  * @param[in]      motor: 电机指针
  * @retval         none
  */
static void plot_motor_position(Motor* motor)
{
    usb_printf("p=%d\r\n", motor->current_position);
}

/**
  * @brief          motor_position任务，用于计算电机的串级PID，对电机进行位置控制
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void motor_position_task(void const * argument)
{
    //wait a time 
    //空闲一段时间
    osDelay(400);
    
    //初始化
    position_init();
    
    const RC_ctrl_t *controller = get_remote_control_point();
    
    int i, j;
    
    while (1)
    {
        
        //遥控器左拨片拨至OFF档可以暂时停止电机的位置环，用于处理紧急情况
        if ((!switch_is_up(controller->rc.s[1])) && (!is_cali_pose))
        {
            for (i = 0; i < MOTOR_GROUP_COUNT; ++i)
            {
                //跳过不使能的电机组
                if (motor_groups[i].enable_group)
                {
                    for (j = 0; j < 4; ++j)
                    {
                        if (motor_groups[i].motor[j].enable_motor)
                        {
                            //如果使能电机，更新电机参数并计算串级PID
                            position_feedback_update(&motor_groups[i].motor[j]);
                            position_control_loop(&motor_groups[i].motor[j]);
                        }
                        else
                        {
                            //如果不使能电机，电流给0
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
                //跳过不使能的电机组
                if (motor_groups[i].enable_group)
                {
                    for (j = 0; j < 4; ++j)
                    {
                        if (motor_groups[i].motor[j].enable_motor)
                        {
                            //如果使能电机，更新电机参数
                            position_feedback_update(&motor_groups[i].motor[j]);
                        }
                        motor_groups[i].motor[j].current = 0;
                    }
                    cmd_motor_group(&motor_groups[i]);
                }
            }
        }
        
        //画电机的位置曲线图，用于调PID
        //plot_motor_position(&motor_groups[0].motor[0]);
        
        osDelay(2);
        
#if INCLUDE_uxTaskGetStackHighWaterMark
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          把电机当前的位置设置为电机位置的原点
  * @param[in]      motor: 电机的指针
  * @retval         none
  */
void set_motor_zero_position(int group, int index)
{
    motor_groups[group].motor[index].current_position = 0;
    motor_groups[group].motor[index].target_position = 0;
}

/**
  * @brief          使能电机的位置环
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @retval         none
  */
void enable_motor_position_control(int group, int index)
{
    motor_groups[group].motor[index].enable_motor = 1;
}

/**
  * @brief          失能电机的位置环
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @retval         none
  */
void disable_motor_position_control(int group, int index)
{
    motor_groups[group].motor[index].enable_motor = 0;
}

/**
  * @brief          通过匀加速运动平稳移动电机(阻塞)
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @param[in]      accel: 加速过程的加速度     (单位: 圈每秒方)
  * @param[in]      decel: 减速过程的减速度     (单位: 圈每秒方)
  * @param[in]      max_speed: 电机最大的速度   (单位: 圈每秒)
  * @param[in]      position: 电机目标位置      (单位: 编码器的值)
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
  * @brief          异步移动电机任务
  * @param[in]      argument: 参数
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
  * @brief          异步(不阻塞)通过匀加速运动平稳移动电机
  * @param[in]      move_args: 电机运动参数的指针
  * @retval         none
  */
void asyn_move_to_position_by_acceleration(move_args* args)
{
    args->is_finished = 0;
    osThreadCreate(osThread(MOVE_MOTOR_THREAD), args);
}

/**
  * @brief          阻塞，等待电机运动结束
  * @param[in]      move_args: 电机运动参数的指针
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
            //跳过不使能的电机组
            if (motor_groups[i].enable_group)
            {
                for (int j = 0; j < 4; ++j)
                {
                    if (motor_groups[i].motor[j].enable_motor)
                    {
                        //如果使能电机，更新电机参数
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
