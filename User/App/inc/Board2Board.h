//
// Created by lemon on 2026/6/20.
//

#ifndef C_BOARD_FRAMEWORK_BOARD2BOARD_H
#define C_BOARD_FRAMEWORK_BOARD2BOARD_H

#include "IMU_Task.h"
#include "DBUS.h"

typedef struct
{
    struct
    {
        IMU_Data_t C_IMU_Data;
        DBUS_Typedef C_DBUS;
        float chassis_vr;
    }gimbal_need;

    struct
    {
        IMU_Data_t P_IMU_Data;
        float gimbal_yaw_rad;
    }chassis_need;
}SEND_DATA_TypDef;

#endif //C_BOARD_FRAMEWORK_BOARD2BOARD_H
