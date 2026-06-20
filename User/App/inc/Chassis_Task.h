#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "All_Init.h"

//底盘应用层参数
typedef struct
{
    float vx;
    float vy;
    float vr;
    float wheel_rmp[4];
}ChassisData_TypDef;

#define REMOTE 0        //遥控器模式宏定义
#define KEY_MOUSE 1     //键鼠模式宏定义


void MOTOR_PID_CHASSIS_INIT();
void chassis_task();
void speed_mapping(ChassisData_TypDef *mapping_data,mecanumInit_typdef mecanumInit_t,DBUS_Typedef DBUS,uint8_t ControlWay);
void MOTOR_PID_CHASSIS_CLT();
void MOTOR_CAN_CHASSIS_SEND();


#endif
