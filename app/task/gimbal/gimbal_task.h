//
// Created by JJ on 2026/11/14.
//

#pragma once
#include "ins/ins.h"
#include "rc/dr16.h"

enum gimbal_mode_e {
    GIMBAL_DISABLE,     // 云台无力，所有控制量置0
    GIMBAL_FOLD,        // pitch 折叠
    GIMBAL_UNFOLD,      // pitch 展开 陀螺仪角度闭环
    GIMBAL_AUTO_AIM,    // 自瞄模式
};

typedef struct {
    struct {
        float yaw; float pitch; float roll;
        float yaw_vel; float pitch_vel; float roll_vel;
    } ins;

    rc::dr16::data_t rc;

    bool gimbal_online;

    gimbal_mode_e gimbal_mode;
    gimbal_mode_e last_gimbal_mode;

    float pitch_target;     // pitch 目标角度
    float yaw_target;         // yaw 目标角度
    float yaw_current;
    float yaw_torque;
}gimbal_t;
extern gimbal_t gimbal;
