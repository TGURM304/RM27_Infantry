//
// Created by JJ on 2026/11/27.
//

#include "vision.h"
#include "bsp/uart.h"
#include "utils/crc.h"

using namespace vision;
using namespace crc16;

PcToGimbal rec_pkg;

void vision::vision_callback(bsp_uart_e device, const uint8_t *data, size_t len) {

}

void vision::init() {
    bsp_uart_set_baudrate(E_UART_10, 1000000);
    bsp_uart_set_callback(E_UART_10, vision::vision_callback);
}
void vision::send(uint8_t mode, const float *q, float yaw, float yaw_vel, float pitch, float pitch_vel) {
    GimbalToPc pkg = {};
    pkg.mode = mode;
    for (int i = 0; i < 4; i++) {
        pkg.q[i] = q[i];
    }
    pkg.yaw = yaw;
    pkg.yaw_vel = yaw_vel;
    pkg.pitch = pitch;
    pkg.pitch_vel = pitch_vel;

    crc16::append(pkg);

    bsp_uart_send(E_UART_10, reinterpret_cast<uint8_t *>(&pkg), sizeof(pkg));
}

vision::PcToGimbal *vision::receive() {
    return &rec_pkg;
}
