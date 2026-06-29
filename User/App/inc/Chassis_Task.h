#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "All_Init.h"
#include "Board2Board.h"

//底盘应用层参数
typedef struct
{
    float vx;
    float vy;
    float vr;

    float vx_real;
    float vy_real;
    float vr_real;

    float vr_follow;
    PID_t vr_follow_PID_P;

    float yaw_encoder;

    float wheel_rmp[4];

}ChassisData_TypDef;

typedef struct
{
    uint8_t offcounter;
    uint8_t isonline;
    uint8_t offline_time;
}remote_linetest;

extern ChassisData_TypDef chassis_data;
extern remote_linetest remote_linecheck;



void MOTOR_PID_CHASSIS_INIT();
void chassis_task();
void speed_mapping(ChassisData_TypDef *mapping_data,mecanumInit_typdef mecanumInit_t,DBUS_Typedef DBUS,uint8_t ControlWay);
void MOTOR_PID_CHASSIS_CLT();
void MOTOR_CAN_CHASSIS_SEND(uint16_t stdid);
void CHASSIS_FOLLOW_CLT();
void GIMBAL_RAD_FORWARD(float t);
uint8_t DBUS_onlinetest();


#endif
