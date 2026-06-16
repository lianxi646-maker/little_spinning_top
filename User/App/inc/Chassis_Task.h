#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "Power_Ctrl.h"
#include "pid_temp.h"
#include "controller.h"
#include "Bottom.h"

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
void speed_mapping(mecanumInit_typdef *mecanumInit_t,DBUS_Typedef DBUS,uint8_t ControlWay);



#endif
