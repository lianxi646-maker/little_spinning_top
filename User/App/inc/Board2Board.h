//
// Created by lemon on 2026/6/20.
//

#ifndef C_BOARD_FRAMEWORK_BOARD2BOARD_H
#define C_BOARD_FRAMEWORK_BOARD2BOARD_H

#include "IMU_Task.h"
#include "DBUS.h"
#include "can_bsp.h"

//收发数据结构体
typedef struct
{
    struct
    {
        IMU_Data_t IMU_Data;
        DBUS_Typedef DBUS;
        float chassis_vr;
        float gimbal_yaw_rad;
        float gimbal_yaw_degree;
    }rx;

    struct
    {
        uint8_t buf_one[8];
        uint8_t buf_two[8];
        uint8_t buf_three[8];
        uint8_t buf_four[8];
    }tx;
}RT_DATA_TypDef;

//float类型接收共用体
typedef union
{
    uint8_t Data[4];
    float Data_f;
}RX_Data_f;
//int16_t类型接收共用体
typedef union
{
    uint8_t Data[2];
    int16_t Data_16;
}RX_Data_16;

typedef union
{
    uint8_t Data;
    uint8_t Data_u8;
}RX_Data_8;

extern RT_DATA_TypDef RT_data;

void Board_to_Board_transmit(RT_DATA_TypDef *TX_data,DBUS_Typedef DBUS_TX, float vr_TX ,IMU_Data_t IMU_Data_TX ,float yaw_rad_TX ,float yaw_deeger_TX);
void Board_to_Board_receive(RT_DATA_TypDef *RX_data ,uint16_t stdid ,uint8_t *rx_data);

#endif //C_BOARD_FRAMEWORK_BOARD2BOARD_H
