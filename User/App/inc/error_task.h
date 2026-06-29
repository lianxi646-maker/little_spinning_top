//
// Created by lemon on 2026/6/29.
//

#ifndef C_BOARD_FRAMEWORK_ERROR_TASK_H
#define C_BOARD_FRAMEWORK_ERROR_TASK_H

#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"

void error_task(uint16_t chassis_stdid ,uint16_t gimbal_stdid);

#endif //C_BOARD_FRAMEWORK_ERROR_TASK_H
