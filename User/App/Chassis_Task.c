#include "Chassis_Task.h"

 ChassisData_TypDef chassis_data = {0};     //定义麦轮各速度

//初始化底盘电机PID
void MOTOR_PID_CHASSIS_INIT()
{
    //初始化底盘四个轮子速度环PID
    float PID_wheel_S[3] = {10,0.0001,0};
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_1.PID_S,16384,7000,PID_wheel_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_2.PID_S,16384,7000,PID_wheel_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_3.PID_S,16384,7000,PID_wheel_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_4.PID_S,16384,7000,PID_wheel_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    //初始化底盘跟随位置环PID
    float PID_follow_P[3]  = {0.1,0.00001,0};
    PID_Init(&chassis_data.vr_follow_PID_P,16384,7000,PID_follow_P,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
}

//底盘电机总任务执行函数
void chassis_task()
{
    //计算底盘跟随所需补偿角速度
    CHASSIS_FOLLOW_CLT();
    //映射当前目标速度
    speed_mapping(&chassis_data,mecanumNumber,DBUS,REMOTE);
    //底盘解算
    MecanumResolve(chassis_data.wheel_rmp,chassis_data.vx_real,chassis_data.vy_real,chassis_data.vr,&mecanumNumber);

    //计算各轮PID输出
    MOTOR_PID_CHASSIS_CLT();

    //CAN发送
    MOTOR_CAN_CHASSIS_SEND();
}

//将遥控器摇杆值映射成速度（mm/s）
void speed_mapping(ChassisData_TypDef *mapping_data,mecanumInit_typdef mecanumInit_t,DBUS_Typedef DBUS,uint8_t ControlWay)
{
    //模式切换
    switch (ControlWay)
    {
        //小陀螺模式
        case 1:
            mapping_data->vx = mecanumInit_t.max_vx_speed/DBUS.Remote.CH3;
            mapping_data->vy = mecanumInit_t.max_vy_speed/DBUS.Remote.CH2;
            mapping_data->vr = mecanumInit_t.max_vw_speed/DBUS.Remote.CH1;

            mapping_data->vx_real = mapping_data->vx * cosf(send_data.chassis_need.gimbal_yaw_rad) + mapping_data->vy * (-sinf(send_data.chassis_need.gimbal_yaw_rad));
            mapping_data->vy_real = mapping_data->vx * sinf(send_data.chassis_need.gimbal_yaw_rad) + mapping_data->vy * cosf(send_data.chassis_need.gimbal_yaw_rad);
            mapping_data->vr_real = mapping_data->vr + mapping_data->vr_follow;
            break;
        //无小陀螺，但保留底盘跟随，云台pitch可动
        case 2:
        mapping_data->vx = mecanumInit_t.max_vx_speed/DBUS.Remote.CH3;
        mapping_data->vy = mecanumInit_t.max_vy_speed/DBUS.Remote.CH2;
        mapping_data->vr = 0;  //相较case 1 的更改点

        mapping_data->vx_real = mapping_data->vx * cosf(send_data.chassis_need.gimbal_yaw_rad) + mapping_data->vy * (-sinf(send_data.chassis_need.gimbal_yaw_rad));
        mapping_data->vy_real = mapping_data->vx * sinf(send_data.chassis_need.gimbal_yaw_rad) + mapping_data->vy * cosf(send_data.chassis_need.gimbal_yaw_rad);
        mapping_data->vr_real = mapping_data->vr + mapping_data->vr_follow;
            break;
        default:
            break;
    }
}

//底盘电机PID计算函数
void MOTOR_PID_CHASSIS_CLT()
{
    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_1.PID_S,ALL_MOTOR.DJI_3508_Chassis_1.DATA.Speed_now,chassis_data.wheel_rmp[0]);
    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_2.PID_S,ALL_MOTOR.DJI_3508_Chassis_2.DATA.Speed_now,chassis_data.wheel_rmp[1]);
    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_3.PID_S,ALL_MOTOR.DJI_3508_Chassis_3.DATA.Speed_now,chassis_data.wheel_rmp[2]);
    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_4.PID_S,ALL_MOTOR.DJI_3508_Chassis_4.DATA.Speed_now,chassis_data.wheel_rmp[3]);
}

//底盘电机CAN发送函数
void MOTOR_CAN_CHASSIS_SEND()
{
    DJI_Current_Ctrl(&hcan1,
                     0x200,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_1.PID_S.Output,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_2.PID_S.Output,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_3.PID_S.Output,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_4.PID_S.Output);
    DJI_Current_Ctrl(&hcan1,
                    0x1FF,
                    0,
                    0,
                    0,
                    0);
}
//计算地盘跟随补偿角速度
void CHASSIS_FOLLOW_CLT()
{
   chassis_data.vr_follow = PID_Calculate(&chassis_data.vr_follow_PID_P,send_data.chassis_need.gimbal_yaw_rad,0);
}
