#include "Chassis_Task.h"

 ChassisData_TypDef chassis_data = {0};     //定义麦轮各速度

//初始化底盘电机PID
void MOTOR_PID_CHASSIS_INIT()
{
    float PID_wheel_S[3] = {10,0.0001,0};
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_1.PID_S,16384,7000,PID_wheel_S,0,0,0,0,0,0);
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_2.PID_S,16384,7000,PID_wheel_S,0,0,0,0,0,0);
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_3.PID_S,16384,7000,PID_wheel_S,0,0,0,0,0,0);
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_4.PID_S,16384,7000,PID_wheel_S,0,0,0,0,0,0);
}

//底盘电机总任务执行函数
void chassis_task()
{
    //计算当前速度
    speed_control(1000,1000,1000,DBUS,REMOTE);
    //计算各轮子所需rmp
    MecanumResolve(chassis_data.wheel_rmp,chassis_data.vx,chassis_data.vy,chassis_data.vr,&mecanumNumber);

    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_1.PID_S,ALL_MOTOR.DJI_3508_Chassis_1.DATA.Speed_now,chassis_data.wheel_rmp[0]);
    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_2.PID_S,ALL_MOTOR.DJI_3508_Chassis_2.DATA.Speed_now,chassis_data.wheel_rmp[1]);
    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_3.PID_S,ALL_MOTOR.DJI_3508_Chassis_3.DATA.Speed_now,chassis_data.wheel_rmp[2]);
    PID_Calculate(&ALL_MOTOR.DJI_3508_Chassis_4.PID_S,ALL_MOTOR.DJI_3508_Chassis_4.DATA.Speed_now,chassis_data.wheel_rmp[3]);

    DJI_Current_Ctrl(&hcan1,
                     0x200,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_1.PID_S.Output,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_2.PID_S.Output,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_3.PID_S.Output,
                     (int16_t)ALL_MOTOR.DJI_3508_Chassis_4.PID_S.Output);
}

//将遥控器摇杆值映射成速度（mm/s）
void speed_control(float vx_max,float vy_max,float vr_max,DBUS_Typedef DBUS,uint8_t ControlWay)
{
    //遥控器模式
    if(ControlWay == 0)
    {
        chassis_data.vx = vx_max/DBUS.Remote.CH3;
        chassis_data.vx = vy_max/DBUS.Remote.CH2;
        chassis_data.vx = vr_max/DBUS.Remote.CH0;
    }
    //键鼠模式（内容先欠着）
    if(ControlWay == 1)
    {

    }
}
