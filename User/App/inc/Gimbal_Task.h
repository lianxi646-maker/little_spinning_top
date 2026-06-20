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

//云台应用层参数
typedef struct
{
    struct
    {
        float yaw_zero;
        float pitch_zero;
    }error;

    struct
    {
        float yaw_rad;
        float yaw_degree;
        float pitch_rad;
        float pitch_degree;
    }angle;

    struct
    {
        float yaw_omega;
        float pitch_omega;
        float yaw_rmp;
        float pitch_rmp;
    }omega;

}GIMBAL_TypDef;

void MOTOR_PID_GIMBAL_INIT();
void Gimbal_angle_init();
void Gimbal_rmp_reslove();
void Gimbal_rmp_mapping();

#endif
