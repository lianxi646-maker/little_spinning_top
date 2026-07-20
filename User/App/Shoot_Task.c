
#include "Shoot_Task.h"

int32_t aim_speed = 6;
Shoot_Ctrl_Block_t shoot_data = {0};

void Shoot_task(float dt)
{
    shoot_state_update();
    Motor_Friction_Control(dt);
    Disc_smooth_contorl(dt);
    Shoot_PID_CLT();
    Shoot_CAN_send(0x200);
}

void Shoot_INIT()
{
    shoot_data.motor_ratio = 36;
    shoot_data.feed_ratio = 2.5f;
    shoot_data.slot_num = 9;
    shoot_data.Lfire_speed = 7000;
    shoot_data.Rfire_speed = -7000;
    shoot_data.dir_sign = -1;
    shoot_data.target_freq = 15;
    shoot_data.Counts_Shoot = shoot_data.motor_ratio  * 8192.0f * shoot_data.feed_ratio / shoot_data.slot_num; //计算拨盘每一发需要的编码器值
    shoot_data.target_rmp = shoot_data.motor_ratio * shoot_data.feed_ratio *(shoot_data.target_freq) * 60;
}

void Shoot_PID_INIT()
{
    float PID_test_friction_r[3] = {15.0f, 0.0001f, 0.0f};
    PID_Init(&ALL_MOTOR.DJI_3508_Shoot_R.PID_S,16384,6000,PID_test_friction_r,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);

    float PID_test_friction_l[3] = {13.0f, 0.0001f, 0.0f};
    PID_Init(&ALL_MOTOR.DJI_3508_Shoot_L.PID_S,16384,6000,PID_test_friction_l,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);

    float PID_test_Disc_P[3] = {0.35f,0.0f,0.0f};
    PID_Init(&ALL_MOTOR.DJI_2006_Disc.PID_P,15000,7000,PID_test_Disc_P,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);

    float PID_test_Disc_S[3] = {15.0f,0.0001f,0.0f};
    PID_Init(&ALL_MOTOR.DJI_2006_Disc.PID_S,10000,5000,PID_test_Disc_S,0.0f,0.0f,0.0f,0.0f,0.0f,Integral_Limit);
}

void Shoot_PID_CLT()
{
    PID_Calculate(&ALL_MOTOR.DJI_3508_Shoot_R.PID_S,ALL_MOTOR.DJI_3508_Shoot_R.DATA.Speed_now,ALL_MOTOR.DJI_3508_Shoot_R.DATA.Aim);
    PID_Calculate(&ALL_MOTOR.DJI_3508_Shoot_L.PID_S,ALL_MOTOR.DJI_3508_Shoot_L.DATA.Speed_now,ALL_MOTOR.DJI_3508_Shoot_L.DATA.Aim);
    PID_Calculate(&ALL_MOTOR.DJI_2006_Disc.PID_P,ALL_MOTOR.DJI_2006_Disc.DATA.Angle_Infinite,ALL_MOTOR.DJI_2006_Disc.DATA.Aim);
    PID_Calculate(&ALL_MOTOR.DJI_2006_Disc.PID_S,ALL_MOTOR.DJI_2006_Disc.DATA.Speed_now,ALL_MOTOR.DJI_2006_Disc.PID_P.Output);
}

void Disc_smooth_contorl(float dt)
{
    static float smooth_ref = 0.0f;
    static float final_target = 0.0f;
    if (shoot_data.shoot_state == true )
    {
        static uint32_t last_shot_time = 0;
        uint32_t now = HAL_GetTick();
        float interval = 1000.0f / shoot_data.target_freq;
        if (now - last_shot_time >= (uint32_t)interval) {
            shoot_data.target_pos_cnt ++;
            last_shot_time = now;
        }

        final_target = (float)shoot_data.target_pos_cnt * shoot_data.Counts_Shoot * (float)shoot_data.dir_sign;
        if (DBUS.Remote.S1 == 2)
        {
            float step = (shoot_data.target_freq * shoot_data.Counts_Shoot) * dt;
            if (smooth_ref > final_target)
            {
                smooth_ref -= step;
                if (smooth_ref < final_target) smooth_ref = final_target;
            } else if (smooth_ref < final_target)
            {
                smooth_ref += step;
                if (smooth_ref > final_target) smooth_ref = final_target;
            }
        }
        else
        {
            smooth_ref = final_target;
        }
    }else if (DBUS.Remote.S1 != 1)
    {
        smooth_ref = final_target;
    }
    shoot_data.Bmotor_Position = smooth_ref;
    ALL_MOTOR.DJI_2006_Disc.DATA.Aim = shoot_data.Bmotor_Position;
}

void shoot_state_update()
{
    static uint8_t last_s1 = 0;
    static uint8_t now_s1 = 0;
    now_s1 = DBUS.Remote.S1;
    static int8_t edge = 0;
    edge = last_s1 - now_s1;
    if ((edge == 2) || (DBUS.Remote.S1 == 2)) {
        shoot_data.shoot_state = true;
    } else {
        shoot_data.shoot_state = false;
    }
    last_s1 = now_s1;
}

void Motor_Friction_Control(float dt)
{
    switch (DBUS.Remote.S2)
    {
    case 1:
        ALL_MOTOR.DJI_3508_Shoot_R.DATA.Aim += shoot_data.Rfire_speed * dt;
        ALL_MOTOR.DJI_3508_Shoot_L.DATA.Aim += shoot_data.Lfire_speed * dt;
        if (ALL_MOTOR.DJI_3508_Shoot_R.DATA.Aim < shoot_data.Rfire_speed)
        {
            ALL_MOTOR.DJI_3508_Shoot_R.DATA.Aim = shoot_data.Rfire_speed;
        }
        if (ALL_MOTOR.DJI_3508_Shoot_L.DATA.Aim > shoot_data.Lfire_speed)
        {
            ALL_MOTOR.DJI_3508_Shoot_L.DATA.Aim = shoot_data.Lfire_speed;
        }
        break;
    case 2:
    case 3:
    default:
        ALL_MOTOR.DJI_3508_Shoot_R.DATA.Aim -= shoot_data.Rfire_speed * dt;
        ALL_MOTOR.DJI_3508_Shoot_L.DATA.Aim -= shoot_data.Lfire_speed * dt;
        if (ALL_MOTOR.DJI_3508_Shoot_R.DATA.Aim > 0)
        {
            ALL_MOTOR.DJI_3508_Shoot_R.DATA.Aim = 0;
        }
        if (ALL_MOTOR.DJI_3508_Shoot_L.DATA.Aim < 0)
        {
            ALL_MOTOR.DJI_3508_Shoot_L.DATA.Aim = 0;
        }
        break;
    }
}

void Shoot_CAN_send(uint16_t stdid)
{
    DJI_Current_Ctrl(&hcan2,
                         stdid,
                         (int16_t)ALL_MOTOR.DJI_3508_Shoot_R.PID_S.Output,
                         (int16_t)ALL_MOTOR.DJI_3508_Shoot_L.PID_S.Output,
                         (int16_t)ALL_MOTOR.DJI_2006_Disc.PID_S.Output,
                         0);
    DJI_Current_Ctrl(&hcan1,
                         stdid,
                         0,
                         0,
                         (int16_t)ALL_MOTOR.DJI_2006_Disc.PID_S.Output,
                         0);
}