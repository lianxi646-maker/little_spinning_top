#include "Gimbal_Task.h"

GIMBAL_TypDef Gimbal_data = {0};

//初始化云台电机PID
void MOTOR_PID_GIMBAL_INIT()
{

    Gimbal_data.error.yaw_zero = ALL_MOTOR.DJI_6020_Yaw.DATA.Angle_Infinite;
    Gimbal_data.error.pitch_zero = ALL_MOTOR.DJI_6020_Pitch.DATA.Angle_Infinite;

    float PID_gimbal_yaw_S[3] = {10,0.0001,0};
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_1.PID_S,27000,7000,PID_gimbal_yaw_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);

    float PID_gimbal_pitch_S[3] = {10,0.0001,0};
    PID_Init(&ALL_MOTOR.DJI_3508_Chassis_1.PID_S,27000,7000,PID_gimbal_pitch_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
}

//将上电时云台yaw轴角度设置为0，pitch轴角度设置为陀螺仪pitch角
void Gimbal_angle_init()
{
    Gimbal_data.angle.yaw_degree = (ALL_MOTOR.DJI_6020_Yaw.DATA.Angle_now - Gimbal_data.error.yaw_zero) * 360.0f / 8192;
    Gimbal_data.angle.yaw_rad = Gimbal_data.angle.yaw_degree/57.3;

    Gimbal_data.angle.pitch_degree = (ALL_MOTOR.DJI_6020_Pitch.DATA.Angle_now - Gimbal_data.error.pitch_zero) * 360.0f / 8192;
    Gimbal_data.angle.pitch_rad = (Gimbal_data.angle.pitch_degree/57.3) + IMU_Data.pitch;
}

//用底盘陀螺仪和云台Pitch轴陀螺仪解算出当前云台两电机的rmp
void Gimbal_rmp_reslove()
{
    Gimbal_data.omega.yaw_omega = -(send_data.pitch_need.C_IMU_Data.gyro[2]) - IMU_Data.gyro[0]*sinf(Gimbal_data.angle.pitch_rad) + IMU_Data.gyro[2]*cosf(Gimbal_data.angle.pitch_rad);
    Gimbal_data.omega.yaw_rmp = Gimbal_data.omega.yaw_omega*9.55f;

    Gimbal_data.omega.pitch_omega = sinf(Gimbal_data.angle.yaw_rad)*send_data.pitch_need.C_IMU_Data.gyro[0] - send_data.pitch_need.C_IMU_Data.gyro[1]*cosf(Gimbal_data.angle.yaw_rad) + IMU_Data.gyro[1];
    Gimbal_data.omega.pitch_rmp = Gimbal_data.omega.pitch_omega*9.55f;
}

//将遥控器值映射到电机目标速度
void Gimbal_rmp_mapping()
{

}