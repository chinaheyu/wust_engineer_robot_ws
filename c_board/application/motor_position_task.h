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
#ifndef MOTOR_POSITION_TASK_H
#define MOTOR_POSITION_TASK_H


#include "CAN_receive.h"

//电机1234与5678对应的标志位
#define MOTOR_ID_1234 0x200
#define MOTOR_ID_5678 0x1FF

//电机组的数量，由于C板只有两个CAN口，故默认四个电机组
#define MOTOR_GROUP_COUNT 4


/* 串级 PID 参数，从左往右依次为 KP KI KD */
const static fp32 speed_pid_params[3] = {5.0f, 0.0f, 1.0f};
const static fp32 position_pid_params[3] = {0.4f, 0.003f, 0.05f};

typedef struct _move_args
{
    int group;
    int index;
    float accel;
    float decel;
    float max_speed;
    int position;
    int is_finished;
} move_args;

/**
  * @brief          设置电机的位置
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @param[in]      position: 电机的位置
  * @retval         none
  */
extern void set_motor_position(int group, int index, int position);

/**
  * @brief          获取电机当前的位置
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @retval         电机当前的位置
  */
extern int get_motor_position(int group, int index);

/**
  * @brief          让所有电机进入快速设置ID模式(已绑定到C板上的按键KEY,按下KEY即可快速设置电机ID)
  * @retval         none
  */
extern void reset_all_motor_ID(void);

/**
  * @brief          把电机当前的位置设置为电机位置的原点
  * @param[in]      motor: 电机的指针
  * @retval         none
  */
extern void set_motor_zero_position(int group, int index);

/**
  * @brief          使能电机的位置环
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @retval         none
  */
void enable_motor_position_control(int group, int index);

/**
  * @brief          失能电机的位置环
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @retval         none
  */
void disable_motor_position_control(int group, int index);

/**
  * @brief          通过匀加速运动平稳移动电机
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @param[in]      accel: 加速过程的加速度     (单位: r/s^2)
  * @param[in]      decel: 减速过程的减速度     (单位: r/s^2)
  * @param[in]      max_speed: 电机最大的速度   (单位: r/s)
  * @param[in]      position: 电机目标位置      (单位: 编码器的值)
  * @retval         none
  */
extern void move_to_position_by_acceleration(int group, int index, float accel, float decel, float max_speed, int position);

/**
  * @brief          异步(不阻塞)通过匀加速运动平稳移动电机
  * @param[in]      group: 电机所属的电机组序号
  * @param[in]      index: 电机的序号
  * @param[in]      accel: 加速过程的加速度     (单位: 圈每秒方)
  * @param[in]      decel: 减速过程的减速度     (单位: 圈每秒方)
  * @param[in]      max_speed: 电机最大的速度   (单位: 圈每秒)
  * @param[in]      position: 电机目标位置      (单位: 编码器的值)
  * @retval         none
  */
void asyn_move_to_position_by_acceleration(move_args* args);

/**
  * @brief          阻塞，等待电机运动结束
  * @param[in]      move_args: 电机运动参数的指针
  * @retval         none
  */
void wait_motor_move_to_position(move_args* args);


extern void cali_pose_hock(void);

// FreeRTOS任务
extern void motor_position_task(void const *pvParameters);


#endif

