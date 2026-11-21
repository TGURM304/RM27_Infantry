//
// Created by JJ on 2026/10/31.
//
#pragma once
#include <cstdint>

struct data_to_chassis {
    int8_t s_l;
    bool gimbal_online;
    float vx, vy, rotate;
    float yaw_motor_angle;



} __attribute__((packed));

struct data_to_gimbal {

} __attribute__((packed));