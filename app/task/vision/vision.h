//
// Created by double_J on 2026/11/27.
//

#pragma once
#include <stdint.h>

#include "bsp/uart.h"

namespace vision {
    struct PcToGimbal {
        uint8_t mode;                 // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
        float yaw_target;             // Yaw 目标角度（弧度）
        float yaw_vel;                // Yaw 角速度
        float yaw_acc;                // Yaw 角加速度
        float pitch_target;           // Pitch 目标角度（弧度）
        float pitch_vel;              // Pitch 角速度
        float pitch_acc;              // Pitch 角加速度
        uint16_t crc16;
        } __attribute__((packed));
    static_assert(sizeof(PcToGimbal) < 64);

    struct GimbalToPc {
        uint8_t mode;       //  0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
        float q[4];         //  wxyz顺序
        float yaw;          // yaw 角度
        float yaw_vel;      // yaw 角速度
        float pitch;        // pitch 角度
        float pitch_vel;    // pitch 角速度
        uint16_t crc16;
    } __attribute__((packed)) ;
    static_assert(sizeof(GimbalToPc) < 64);

    void init();
    void send(uint8_t mode, const float *q, float yaw, float yaw_vel, float pitch, float pitch_vel);
    PcToGimbal *receive();
    void vision_callback(bsp_uart_e device, const uint8_t *data, size_t len);

}

