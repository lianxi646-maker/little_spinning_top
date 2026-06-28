#include "Gimbal_Task.h"

GIMBAL_TypDef Gimbal_data = {0};

void Gimbal_task()
{

    //更新云台角度
    Gimbal_angle_update();

    //解算云台电机转速
    Gimbal_rmp_reslove();

    //解算云台自稳补偿角速度并结合遥控器值映射到电机目标速度
    Gimbal_rmp_mapping(0.5f,0.5f,6000.0f,2000.0f,RT_data.rx.DBUS.Remote.S1);

    //计算云台电机PID输出
    MOTOR_GIMBAL_CLT();

    //CAN发送
    MOTOR_GIMBAL_CAN_SEND(RT_data.rx.DBUS.Remote.S2);


}

//初始化云台电机PID与上电时角度静差
void MOTOR_PID_GIMBAL_INIT()
{
    // //初始化上电时电机角度误差
    // Gimbal_data.error.yaw_zero = ALL_MOTOR.m_dm4310_y_t.DATA.Angle_Infinite;
    // Gimbal_data.error.pitch_zero = ALL_MOTOR.m_dm4310_p_t.DATA.Angle_Infinite;
    //初始化云台pitch编码器中值
    Gimbal_data.angle.pitch.encoder_mid = 4000;
    //初始化云台yaw轴PID
    float PID_gimbal_yaw_S[3] = {10,0.0001,0};
    float PID_gimbal_yaw_P[3] = {3,0,0};
    PID_Init(&ALL_MOTOR.m_dm4310_y_t.PID_S,27000,7000,PID_gimbal_yaw_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&ALL_MOTOR.m_dm4310_y_t.PID_P,200,30,PID_gimbal_yaw_P,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    //初始化云台pitch轴PID
    float PID_gimbal_pitch_P[3] = {3,0,0};
    float PID_gimbal_pitch_S[3] = {10,0.0001,0};
    PID_Init(&ALL_MOTOR.m_dm4310_p_t.PID_S,27000,7000,PID_gimbal_pitch_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
    PID_Init(&ALL_MOTOR.m_dm4310_p_t.PID_P,200,30,PID_gimbal_pitch_P,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
}

//将上电时云台yaw轴角度设置为0，pitch轴角度设置为陀螺仪pitch角
void Gimbal_angle_update()
{
    //更新云台yaw相对于底盘系角度   fomd()为取模函数
    //Gimbal_data.angle.yaw.degree = (ALL_MOTOR.m_dm4310_y_t.DATA.Angle_Infinite - Gimbal_data.error.yaw_zero) * 360.0f / 8192;
    Gimbal_data.angle.yaw.rad_real = ALL_MOTOR.m_dm4310_y_t.DATA.ralativeAngle / 57.3;
    Gimbal_data.angle.yaw.rad = fmod(Gimbal_data.angle.yaw.rad_real, 6.28f);
    //将云台yaw相对于底盘系的角度归化为[-Π,Π]
    if (Gimbal_data.angle.yaw.rad > 3.14f) Gimbal_data.angle.yaw.rad -= 6.28f;
    if (Gimbal_data.angle.yaw.rad < -3.14f) Gimbal_data.angle.yaw.rad += 6.28f;

    //更新云台pitch轴相对于底盘角度
    Gimbal_data.angle.pitch.degree = (ALL_MOTOR.m_dm4310_p_t.DATA.Angle_Infinite - Gimbal_data.angle.pitch.encoder_mid) * 360.0f / 8192;
    Gimbal_data.angle.pitch.rad_Infinite = Gimbal_data.angle.pitch.degree/57.3;
    Gimbal_data.angle.pitch.rad = fmod(Gimbal_data.angle.pitch.rad_Infinite, 6.28f);
    //将云台pitch相对于底盘系的角度归化为[-Π,Π]
    if (Gimbal_data.angle.pitch.rad > 3.14f) Gimbal_data.angle.pitch.rad -= 6.28f;
    if (Gimbal_data.angle.pitch.rad < -3.14f) Gimbal_data.angle.pitch.rad += 6.28f;
}

//用底盘陀螺仪和云台Pitch轴陀螺仪解算出当前云台两电机的rpm
void Gimbal_rmp_reslove()
{
    //解算云台yaw轴相对底盘系rpm
    Gimbal_data.omega.yaw.omega_now = -(RT_data.rx.IMU_Data.gyro[2]) - IMU_Data.gyro[0]*sinf(Gimbal_data.angle.pitch.rad) + IMU_Data.gyro[2]*cosf(Gimbal_data.angle.pitch.rad);
    Gimbal_data.omega.yaw.rmp_now = Gimbal_data.omega.yaw.omega_now*9.55f;
    //解算云台pitch轴相对于底盘系rpm
    Gimbal_data.omega.pitch.omega_now = sinf(Gimbal_data.angle.yaw.rad)*RT_data.rx.IMU_Data.gyro[0] - RT_data.rx.IMU_Data.gyro[1]*cosf(Gimbal_data.angle.yaw.rad) + IMU_Data.gyro[1];
    Gimbal_data.omega.pitch.rmp_now = Gimbal_data.omega.pitch.omega_now*9.55f;
}

//将遥控器值映射到电机目标速度
void Gimbal_rmp_mapping(float yaw_omega_max, float pitch_omega_max, float pitch_angle_max , float pitch_angle_min, uint8_t mod)
{
    //更新云台自稳补偿角速度
    Gimbal_data.omega.yaw.omega_compensation = -RT_data.rx.chassis_vr;
    Gimbal_data.omega.pitch.omega_compensation = (sinf(Gimbal_data.angle.yaw.rad) * RT_data.rx.IMU_Data.gyro[0] - cosf(Gimbal_data.angle.yaw.rad) * RT_data.rx.IMU_Data.gyro[1]);
    //控制模式状态机
    switch (mod)
    {
        //小陀螺模式，云台oitch固定为中值
        case 1:
            //解算云台yaw真正的角速度与双环角度目标
            Gimbal_data.omega.yaw.omega_remote = RT_data.rx.DBUS.Remote.CH0 * yaw_omega_max / 660.0;
            Gimbal_data.angle.yaw.rad_target += (Gimbal_data.omega.yaw.omega_remote + Gimbal_data.omega.yaw.omega_compensation) * 0.0001;
            ALL_MOTOR.m_dm4310_y_t.DATA.Aim = Gimbal_data.angle.yaw.rad_target * 8192 / 6.28f;
            //保持pitch轴在中值
            Gimbal_data.angle.pitch.rad_target += Gimbal_data.omega.pitch.omega_compensation * 0.0001;
            ALL_MOTOR.m_dm4310_p_t.DATA.Aim = Gimbal_data.angle.pitch.encoder_mid + Gimbal_data.angle.pitch.rad_target * 8192 / 6.28f;
            break;
        //无小陀螺，但保留底盘跟随，云台pitch可动
        case 2:
            //解算云台yaw真正的角速度与双环角度目标
            Gimbal_data.omega.yaw.omega_remote = RT_data.rx.DBUS.Remote.CH0 * yaw_omega_max / 660.0;
            Gimbal_data.angle.yaw.rad_target += (Gimbal_data.omega.yaw.omega_remote + Gimbal_data.omega.yaw.omega_compensation) * 0.0001;
            ALL_MOTOR.m_dm4310_y_t.DATA.Aim = Gimbal_data.angle.yaw.rad_target * 8192 / 6.28f;
            if (ALL_MOTOR.m_dm4310_p_t.DATA.Aim > pitch_angle_max) Gimbal_data.angle.pitch.rad_target = pitch_angle_max;
            if (ALL_MOTOR.m_dm4310_p_t.DATA.Aim < pitch_angle_min) Gimbal_data.angle.pitch.rad_target = pitch_angle_min;
            //解算云台pitch真正的角速度与双环角度目标
            Gimbal_data.omega.pitch.omega_remote = RT_data.rx.DBUS.Remote.CH1 * pitch_omega_max / 660.0;
            Gimbal_data.angle.pitch.rad_target += (Gimbal_data.omega.pitch.omega_remote + Gimbal_data.omega.pitch.omega_compensation) * 0.0001;
            ALL_MOTOR.m_dm4310_p_t.DATA.Aim = Gimbal_data.angle.pitch.rad_target * 8192 / 6.28f;
            if (ALL_MOTOR.m_dm4310_p_t.DATA.Aim > pitch_angle_max) Gimbal_data.angle.pitch.rad_target = pitch_angle_max;
            if (ALL_MOTOR.m_dm4310_p_t.DATA.Aim < pitch_angle_min) Gimbal_data.angle.pitch.rad_target = pitch_angle_min;
            break;
        default:
            break;
    }
}
//计算云台两电机的输出
void MOTOR_GIMBAL_CLT()
{
    PID_Calculate(&ALL_MOTOR.m_dm4310_y_t.PID_P,ALL_MOTOR.m_dm4310_y_t.DATA.Angle_Infinite, ALL_MOTOR.m_dm4310_y_t.DATA.Aim);
    PID_Calculate(&ALL_MOTOR.m_dm4310_y_t.PID_S,Gimbal_data.omega.yaw.rmp_now,ALL_MOTOR.m_dm4310_y_t.PID_P.Output);

    PID_Calculate(&ALL_MOTOR.m_dm4310_p_t.PID_P,ALL_MOTOR.m_dm4310_p_t.DATA.Angle_Infinite, ALL_MOTOR.m_dm4310_p_t.DATA.Aim);
    PID_Calculate(&ALL_MOTOR.m_dm4310_p_t.PID_S,Gimbal_data.omega.pitch.rmp_now,ALL_MOTOR.m_dm4310_p_t.PID_P.Output);
}
//云台电机CAN发送
void MOTOR_GIMBAL_CAN_SEND(uint8_t mod)
{
    switch (mod)
    {
    case 1:
    case 2:
        DJI_Current_Ctrl(&hcan2,
                        0x300,
                        (int16_t)ALL_MOTOR.m_dm4310_y_t.PID_S.Output,
                        (int16_t)ALL_MOTOR.m_dm4310_p_t.PID_S.Output,
                        0,
                        0);
        DJI_Current_Ctrl(&hcan2,
                        0x2FE,
                        0,
                        0,
                        0,
                        0);
        break;
    default:
        break;
    }
}