//
// Created by lemon on 2026/6/20.
//

#include "Board2Board.h"

//定义收发类型变量
RT_DATA_TypDef RT_data = {0};

//发送函数
void Board_to_Board_transmit(RT_DATA_TypDef *TX_data,DBUS_Typedef DBUS_TX, float vr_TX ,IMU_Data_t IMU_Data_TX ,float yaw_rad_TX)
{
    //将int16_t类型数据转化为uint16_t类型数据（二进制编码无变化（只适用于相同字节的整形，不同字节或同字节不同类型均会改变二进制编码））
    uint16_t ch0 = (uint16_t)DBUS_TX.Remote.CH0;
    uint16_t ch1 = (uint16_t)DBUS_TX.Remote.CH1;
    //将高字节二进制编码通过位运算填充到有八位的一字节数组中;
    //拆分遥控器ch0,ch1数据与底盘角速度
    TX_data->tx.buf_one[0] = ch0 & 0xFF;
    TX_data->tx.buf_one[1] = (ch0>> 8) & 0xFF;
    TX_data->tx.buf_one[2] = ch1 & 0xFF;
    TX_data->tx.buf_one[3] = (ch1>> 8) & 0xFF;
    memcpy(TX_data->tx.buf_one + 4 , &vr_TX, 4);

    //拆分陀螺仪下，y角速度
    memcpy(TX_data->tx.buf_two ,&IMU_Data_TX.gyro[0], 4);
    memcpy(TX_data->tx.buf_two + 4, &IMU_Data_TX.gyro[1], 4);
    //拆分陀螺仪z角速度与云台yaw轴相对于底盘的弧度制角度
    memcpy(TX_data->tx.buf_three ,&IMU_Data_TX.gyro[2], 4);
    memcpy(TX_data->tx.buf_three + 4, &yaw_rad_TX, 4);

    TX_data->tx.buf_four[0] = DBUS_TX.Remote.S2 & 0xFF;
    TX_data->tx.buf_four[1] = DBUS_TX.Remote.S1 & 0xFF;
    //CAN发送所有数据
    //由于CAN一帧只能发送八字节数据，所以分三帧发送
    canx_send_data(&hcan1, 0x226, TX_data->tx.buf_one);
    canx_send_data(&hcan1, 0x227, TX_data->tx.buf_two);
    canx_send_data(&hcan1, 0x228, TX_data->tx.buf_three);
    canx_send_data(&hcan1, 0x225, TX_data->tx.buf_four);
}
void Board_to_Board_receive(RT_DATA_TypDef *RX_data ,uint16_t stdid ,uint8_t *rx_data)
{
    //定义接收类型共用体变量（每次退出函数清零）
    //共用体可以平滑转换数据类型且不改变二进制编码
    RX_Data_f F_data = {0};
    RX_Data_16 i16_data = {0};
    RX_Data_8 u8_data = {0};
    //根据帧头选择解析的数据
    switch (stdid)
    {
        case 0x225:
            //解算遥控器S1,S2数据
            u8_data.Data = rx_data[0];
            RX_data->rx.DBUS.Remote.S2 = u8_data.Data_u8;
            u8_data.Data = rx_data[1];
            RX_data->rx.DBUS.Remote.S1 = u8_data.Data_u8;
            break;
        case 0x226:
            //解算ch0数据
            i16_data.Data[0] = rx_data[0];
            i16_data.Data[1] = rx_data[1];
            RX_data->rx.DBUS.Remote.CH0 = i16_data.Data_16;
            //解算ch1数据
            i16_data.Data[0] = rx_data[2];
            i16_data.Data[1] = rx_data[3];
            RX_data->rx.DBUS.Remote.CH1 = i16_data.Data_16;
            //解算底盘角速度
            F_data.Data[0] = rx_data[4];
            F_data.Data[1] = rx_data[5];
            F_data.Data[2] = rx_data[6];
            F_data.Data[3] = rx_data[7];
            RX_data->rx.chassis_vr = F_data.Data_f;
            break;
        case 0x227:
            //解算陀螺仪x角速度
            F_data.Data[0] = rx_data[0];
            F_data.Data[1] = rx_data[1];
            F_data.Data[2] = rx_data[2];
            F_data.Data[3] = rx_data[3];
            RX_data->rx.IMU_Data.gyro[0] = F_data.Data_f;
            //解算陀螺仪y角速度
            F_data.Data[0] = rx_data[4];
            F_data.Data[1] = rx_data[5];
            F_data.Data[2] = rx_data[6];
            F_data.Data[3] = rx_data[7];
            RX_data->rx.IMU_Data.gyro[1] = F_data.Data_f;
            break;
        case 0x228:
            //解算陀螺仪z角速度
            F_data.Data[0] = rx_data[0];
            F_data.Data[1] = rx_data[1];
            F_data.Data[2] = rx_data[2];
            F_data.Data[3] = rx_data[3];
            RX_data->rx.IMU_Data.gyro[2] = F_data.Data_f;
            //解算云台yaw轴相对于底盘的弧度制角度
            F_data.Data[0] = rx_data[4];
            F_data.Data[1] = rx_data[5];
            F_data.Data[2] = rx_data[6];
            F_data.Data[3] = rx_data[7];
            RX_data->rx.gimbal_yaw_rad = F_data.Data_f;
            break;
        default:
            break;
    }
}
