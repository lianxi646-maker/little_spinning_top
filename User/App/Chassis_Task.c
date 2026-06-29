#include "Chassis_Task.h"

 ChassisData_TypDef chassis_data = {0};     //定义麦轮各速度
remote_linetest remote_linecheck = {0};  //定义遥控器通信检测结构体

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
    //云台yaw轴弧度制角度前馈
    GIMBAL_RAD_FORWARD(0.0001f);
    //计算底盘跟随所需补偿角速度
    CHASSIS_FOLLOW_CLT();
    //映射当前目标速度
    speed_mapping(&chassis_data,mecanumNumber,DBUS,DBUS.Remote.S2);
    //底盘解算
    MecanumResolve(chassis_data.wheel_rmp,chassis_data.vx_real,chassis_data.vy_real,chassis_data.vr_real,&mecanumNumber);

    //计算各轮PID输出
    MOTOR_PID_CHASSIS_CLT();

    //CAN发送
    MOTOR_CAN_CHASSIS_SEND(0x200);
}

//映射速度（mm/s）
void speed_mapping(ChassisData_TypDef *mapping_data,mecanumInit_typdef mecanumInit_t,DBUS_Typedef DBUS,uint8_t ControlWay)
{
    //模式切换
    switch (ControlWay)
    {
        //小陀螺模式
        case 1:
            //将遥控器摇杆值映射成速度（mm/s）(rad/s)
            mapping_data->vx = DBUS.Remote.CH3 * mecanumInit_t.max_vx_speed / 660.0;
            mapping_data->vy = DBUS.Remote.CH2 * mecanumInit_t.max_vy_speed / 660.0;
            mapping_data->vr = DBUS.Remote.CH1 * mecanumInit_t.max_vw_speed / 660.0;
            //加入底盘跟随与小陀螺后的实际目标速度（mm/s）(rad/s)
            mapping_data->vx_real = mapping_data->vx * cosf(RT_data.rx.gimbal_yaw_rad) + mapping_data->vy * (-sinf(RT_data.rx.gimbal_yaw_rad));
            mapping_data->vy_real = mapping_data->vx * sinf(RT_data.rx.gimbal_yaw_rad) + mapping_data->vy * cosf(RT_data.rx.gimbal_yaw_rad);
            mapping_data->vr_real = mapping_data->vr + mapping_data->vr_follow;
            break;
        //无小陀螺，但保留底盘跟随，云台pitch可动
        case 2:
        mapping_data->vx = DBUS.Remote.CH3 * mecanumInit_t.max_vx_speed / 660.0;
        mapping_data->vy = DBUS.Remote.CH2 * mecanumInit_t.max_vy_speed / 660.0;
        mapping_data->vr = 0;  //相较case 1 的更改点

        mapping_data->vx_real = mapping_data->vx * cosf(RT_data.rx.gimbal_yaw_rad) + mapping_data->vy * (-sinf(RT_data.rx.gimbal_yaw_rad));
        mapping_data->vy_real = mapping_data->vx * sinf(RT_data.rx.gimbal_yaw_rad) + mapping_data->vy * cosf(RT_data.rx.gimbal_yaw_rad);
        mapping_data->vr_real = mapping_data->vr + mapping_data->vr_follow;
            break;
        default:
        error_task(0x200,0x300);
            break;
    }
}
//云台yaw轴相对于底盘弧度制角度前馈
void GIMBAL_RAD_FORWARD(float t)
{
    RT_data.rx.gimbal_yaw_degree += chassis_data.vr_real * t ;
    RT_data.rx.gimbal_yaw_rad = RT_data.rx.gimbal_yaw_degree / 57.3f;
    chassis_data.yaw_encoder = RT_data.rx.gimbal_yaw_degree /0.044f;
}
//计算底盘跟随所需速度
void CHASSIS_FOLLOW_CLT()
{
    chassis_data.vr_follow = PID_Calculate(&chassis_data.vr_follow_PID_P,chassis_data.yaw_encoder,0);
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
void MOTOR_CAN_CHASSIS_SEND(uint16_t stdid)
{
        DJI_Current_Ctrl(&hcan2,
                         stdid,
                         (int16_t)ALL_MOTOR.DJI_3508_Chassis_1.PID_S.Output,
                         (int16_t)ALL_MOTOR.DJI_3508_Chassis_2.PID_S.Output,
                         (int16_t)ALL_MOTOR.DJI_3508_Chassis_3.PID_S.Output,
                         (int16_t)ALL_MOTOR.DJI_3508_Chassis_4.PID_S.Output);
}

uint8_t DBUS_onlinetest()
{

    if (remote_linecheck.offcounter >= DBUS.ONLINE_JUDGE_TIME)
    {
        remote_linecheck.offcounter = DBUS.ONLINE_JUDGE_TIME;
        return 0;
    }
    else
    {
        return 1;
    }
    return 0;
}

