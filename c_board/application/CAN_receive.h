/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "can.h"
#include "pid.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct _Motor
{
    //�Ƿ�ʹ�ܵ��
    int enable_motor;
    
    //����ľ��Ա�������Χ��0 ~ ecd_range.
    int ecd_range;
    
    //�����ǰ��������ֵ
    int ecd;
    
    //�����ǰ���ٶ�
    float speed;
    
    //����ϴεı�����ֵ
    int offset_ecd;
    
    //�����ǰ���ܱ�����ֵ
    int current_position;
    
    //�������ָ��
    const motor_measure_t *motor_measure;
    
    //���õ���ĵ���
    int16_t current;
    
    //�����������Ŀ��ֵ
    int target_position;
    
    //����ٶȵ�Ŀ��ֵ
    float target_speed;
    
    //����ٶȻ���pid
    pid_type_def speed_pid;
    
    //���λ�û���pid
    pid_type_def position_pid;
    
    
    
} Motor;

typedef struct _MotorGroup
{
    //�Ƿ�ʹ�ܵ����
    int enable_group;
    
    //������Ӧ��can��
    CAN_HandleTypeDef *hcan_ptr;
    
    //������id��1234����5678
    uint32_t motor_id;
    
    //�ĸ�����ṹ��
    Motor motor[4];
    
} MotorGroup;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          �õ������������е����е�������������IDģʽ
  * @param[in]      none
  * @retval         none
  */
extern void cmd_group_reset_ID(MotorGroup* motor_group);

/**
  * @brief          ��������еĵ�����͵�������ָ��
  * @param[in]      motor_group: ������ָ��
  * @retval         none
  */
extern void cmd_motor_group(MotorGroup* motor_group);

/**
  * @brief          ���ص������ָ��
  * @param[in]      motor_group: �����ָ��
  * @param[in]      motor_id: ����ĵ��ID ( 1-8 )
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_motor_measure_point(MotorGroup* motor_group, int motor_id);



/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
