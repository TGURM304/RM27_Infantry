//
// Created by JJ on 2026/11/15.
//
#include "shoot_task.h"

#include <cstdio>

#include "bsp/uart.h"
#include "utils/os.h"
#include "motor/dji.h"
#include "controller/pid.h"
#include "filter/lowpass_filter.h"
#include "utils/vofa.h"

using namespace motor;
using namespace controller;
using namespace filter;

dji ammo_left("ammo_left", dji::M3508,dji::param_t{.id = 1,.port = E_CAN_2,.mode = dji::CURRENT}, -1, 1);
dji ammo_right("ammo_right", dji::M3508,dji::param_t{.id = 2,.port = E_CAN_2,.mode = dji::CURRENT}, -1, 1);

pid ammo_left_pid(750, 20, 5, 16384, 16384);
pid ammo_right_pid(750, 20, 5, 16384, 16384);

dji trigger_motor("trigger", dji::M2006,dji::param_t{.id = 5,.port = E_CAN_1,.mode = dji::CURRENT}, -1);
pid trigger_pid(700, 1, 1, 16384, 16384);
low_pass_filter trigger_filter(30.f, 1000.f);

float spd = 0;
[[noreturn]] void shoot_task(void *args) {
    // ammo_left.init();
    // ammo_right.init();
    //
    // trigger_motor.init();
    // bsp_uart_set_callback(E_UART_1, [](bsp_uart_e device, const uint8_t *data, size_t len) {
    //     sscanf(reinterpret_cast<const char *>(data), "%f", &spd);
    // });
    while (true) {
        ammo_left.update(ammo_left_pid(ammo_left.feedback.speed, 40));
        ammo_right.update(ammo_right_pid(ammo_right.feedback.speed, -40));

        trigger_motor.update(trigger_pid(trigger_filter.update(trigger_motor.feedback.speed), spd));
        // vofa::send(E_UART_1, ammo_left.feedback.speed, ammo_right.feedback.speed);
        // vofa::send(E_UART_1, spd, trigger_motor.feedback.speed, trigger_motor.feedback.timestamp);
        // vofa::send(E_UART_1, trigger_motor.feedback.timestamp, spd, trigger_motor.feedback.speed);
        os::task::sleep(1);
    }
}