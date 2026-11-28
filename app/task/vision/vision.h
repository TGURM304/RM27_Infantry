//
// Created by double_J on 2026/11/27.
//

#pragma once
#include <stdint.h>

#include "bsp/uart.h"

namespace vision {
    struct PcToGimbal {
        uint8_t mode;                 // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
        float yaw_target;             // Yaw角度（弧度）
        float yaw_vel;                // Yaw角速度
        float yaw_acc;                // Yaw角加速度
        float pitch_target;           // Pitch角度（弧度）
        float pitch_vel;              // Pitch角速度
        float pitch_acc;              // Pitch角加速度
        } __attribute__((packed));

    struct GimbalToPc {
        uint8_t mode;   //  0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
        float q[4];     //  wxyz顺序
        float yaw;
        float yaw_vel;
        float pitch;
        float pitch_vel;
        uint16_t crc16;
    } __attribute__((packed)) ;


    void init();
    void send(uint8_t mode, const float *q, float yaw, float yaw_vel, float pitch, float pitch_vel);
    PcToGimbal *receive();
    void vision_callback(bsp_uart_e device, const uint8_t *data, size_t len);

}

