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
#ifndef MOTOR_POSITION_TASK_H
#define MOTOR_POSITION_TASK_H


#include "CAN_receive.h"

//���1234��5678��Ӧ�ı�־λ
#define MOTOR_ID_1234 0x200
#define MOTOR_ID_5678 0x1FF

//����������������C��ֻ������CAN�ڣ���Ĭ���ĸ������
#define MOTOR_GROUP_COUNT 4


/* ���� PID ������������������Ϊ KP KI KD */
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
  * @brief          ���õ����λ��
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @param[in]      position: �����λ��
  * @retval         none
  */
extern void set_motor_position(int group, int index, int position);

/**
  * @brief          ��ȡ�����ǰ��λ��
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @retval         �����ǰ��λ��
  */
extern int get_motor_position(int group, int index);

/**
  * @brief          �����е�������������IDģʽ(�Ѱ󶨵�C���ϵİ���KEY,����KEY���ɿ������õ��ID)
  * @retval         none
  */
extern void reset_all_motor_ID(void);

/**
  * @brief          �ѵ����ǰ��λ������Ϊ���λ�õ�ԭ��
  * @param[in]      motor: �����ָ��
  * @retval         none
  */
extern void set_motor_zero_position(int group, int index);

/**
  * @brief          ʹ�ܵ����λ�û�
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @retval         none
  */
void enable_motor_position_control(int group, int index);

/**
  * @brief          ʧ�ܵ����λ�û�
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @retval         none
  */
void disable_motor_position_control(int group, int index);

/**
  * @brief          ͨ���ȼ����˶�ƽ���ƶ����
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @param[in]      accel: ���ٹ��̵ļ��ٶ�     (��λ: r/s^2)
  * @param[in]      decel: ���ٹ��̵ļ��ٶ�     (��λ: r/s^2)
  * @param[in]      max_speed: ��������ٶ�   (��λ: r/s)
  * @param[in]      position: ���Ŀ��λ��      (��λ: ��������ֵ)
  * @retval         none
  */
extern void move_to_position_by_acceleration(int group, int index, float accel, float decel, float max_speed, int position);

/**
  * @brief          �첽(������)ͨ���ȼ����˶�ƽ���ƶ����
  * @param[in]      group: ��������ĵ�������
  * @param[in]      index: ��������
  * @param[in]      accel: ���ٹ��̵ļ��ٶ�     (��λ: Ȧÿ�뷽)
  * @param[in]      decel: ���ٹ��̵ļ��ٶ�     (��λ: Ȧÿ�뷽)
  * @param[in]      max_speed: ��������ٶ�   (��λ: Ȧÿ��)
  * @param[in]      position: ���Ŀ��λ��      (��λ: ��������ֵ)
  * @retval         none
  */
void asyn_move_to_position_by_acceleration(move_args* args);

/**
  * @brief          �������ȴ�����˶�����
  * @param[in]      move_args: ����˶�������ָ��
  * @retval         none
  */
void wait_motor_move_to_position(move_args* args);


extern void cali_pose_hock(void);

// FreeRTOS����
extern void motor_position_task(void const *pvParameters);


#endif

