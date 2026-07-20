#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "DBUS.h"

typedef struct
{
    float motor_ratio;//电机减速比
    float feed_ratio;//齿轮减速比
    float slot_num;//弹槽数
    float Counts_Shoot;//电机减速比*齿轮减速比/弹槽数
    float Lfire_speed;
    float Rfire_speed;
    int8_t dir_sign ;//方向标志位
    float target_freq;
    float target_rmp;
    int64_t target_pos_cnt;
    float Bmotor_Position;
    bool shoot_state;
}Shoot_Ctrl_Block_t;

extern Shoot_Ctrl_Block_t shoot_data;

void Shoot_INIT();
void Shoot_PID_INIT();
void Shoot_PID_CLT();
void Motor_Friction_Control(float dt);
void Shoot_task(float dt);
void Shoot_CAN_send(uint16_t stdid);
void Disc_smooth_contorl(float dt);
void shoot_state_update();



#endif
