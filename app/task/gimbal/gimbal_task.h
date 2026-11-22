//
// Created by JJ on 2026/11/14.
//

#pragma once
#include "rc/dr16.h"

#define GIMBAL_MASS 1.564                 // 云台质量

#define FOLD_MOTOR_DOWN_LIMIT 0.0        // fold motor 折叠角度
#define FOLD_MOTOR_UP_LIMIT -1.1414947    // fold motor 展开角度

#define PITCH_MOTOR_DOWN_LIMIT 0.43888  // pitch motor 低头角度
#define PITCH_MOTOR_UP_LIMIT -0.175255   // pitch motor 抬头角度

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

    float yaw_rc_delta;

    float pitch_target;       // pitch 目标角度
    float yaw_target;         // yaw 目标角度

    float pitch_current;      // pitch 当前角度
    float yaw_current;        // yaw 当前角度

    // 控制量
    float fold_motor_target_angle;
    float pitch_torque;       // pitch 目标力矩
    float yaw_torque;         // yaw 目标力矩
}gimbal_t;
extern gimbal_t gimbal;
