//
// Created by lemon on 2026/6/29.
//
#include "error_task.h"

//检测异常失能函数
void error_task(uint16_t chassis_stdid ,uint16_t gimbal_stdid)
{
    DJI_Current_Ctrl(&hcan2,
                         chassis_stdid,
                         0,
                         0,
                         0,
                         0);
    DM_Motor_Send(&hcan2,
                        0x300,
                        0,
                        0,
                        0,
                        0);
}