//
// Created by lemon on 2026/6/20.
//

#ifndef C_BOARD_FRAMEWORK_BOARD2BOARD_H
#define C_BOARD_FRAMEWORK_BOARD2BOARD_H

#include "IMU_Task.h"

typedef struct
{
    struct
    {
        IMU_Data_t C_IMU_Data;
    }pitch_need;

    struct
    {
        IMU_Data_t P_IMU_Data;
    }chassis_need;
}SEND_DATA_TypDef;

#endif //C_BOARD_FRAMEWORK_BOARD2BOARD_H
