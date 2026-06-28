#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "IMU_Task.h"
#include "All_Init.h"
#include "WHW_IRQN.h"
#include "controller.h"
#include "pid_temp.h"
#include <arm_math.h>
#include "Board2Board.h"

//云台应用层参数
typedef struct
{
    //云台上电时标零的差值
    struct
    {
        float yaw_zero;
        float pitch_zero;
    }error;
    //云台各角度
    struct
    {
        struct
        {
            float rad;              //规划到[-Π,Π]的角度
            float rad_real;     //未规划的连续弧度制角度
            float rad_target;       //弧度制角度目标值
            float degree;           //未规划的连续角度制角度
        }yaw;
        struct
        {
            float rad;
            float rad_Infinite;
            float rad_target;
            float degree;
            float encoder_mid;
        }pitch;
    }angle;
    //云台各角速度
    struct
    {
        struct
        {
            float rmp_now;              //编码器当前速度
            float omega_now;            //当前角速度
            float omega_remote;         //遥控器期望角速度（不包含自稳补偿）
            float omega_compensation;   //自稳补偿角速度
        }yaw;
        struct
        {
            float rmp_now;
            float omega_now;
            float omega_remote;
            float omega_compensation;
        }pitch;
    }omega;

}GIMBAL_TypDef;

extern GIMBAL_TypDef Gimbal_data;

void MOTOR_PID_GIMBAL_INIT();
void Gimbal_angle_update();
void Gimbal_rmp_reslove();
void Gimbal_rmp_mapping(float yaw_omega_max, float pitch_omega_max, float pitch_angle_max , float pitch_angle_min, uint8_t mod);
void MOTOR_GIMBAL_CLT();
void Gimbal_task();
void MOTOR_GIMBAL_CAN_SEND(uint8_t mod);

#endif
