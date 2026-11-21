//
// Created by JJ on 2026/10/31.
//
#include "chassis_task.h"

#include <iso646.h>

#include "def.h"

#include "utils/os.h"
#include "motor/dji.h"
#include "motor/dm.h"
#include "controller/pid.h"
#include "utils/msg.h"
#include "utils/vofa.h"

constexpr float SQRT2 = M_SQRT2;
constexpr float fpi = M_PI;

using namespace motor;
using namespace controller;
msg::can_receiver<data_to_chassis, E_CAN_3, 0x311> chassis_rx;
msg::can_sender<data_to_gimbal, E_CAN_3, 0x322> chassis_tx;

dji motor_1("FL", dji::M3508,dji::param_t{.id = 1,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
dji motor_2("BL", dji::M3508,dji::param_t{.id = 2,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
dji motor_3("BR", dji::M3508,dji::param_t{.id = 3,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
dji motor_4("FR", dji::M3508,dji::param_t{.id = 4,.port = E_CAN_1,.mode = dji::CURRENT}, -1, 268.f / 17.f);
pid speed_pid1(750, 20, 5, 16384, 16384);
pid speed_pid2(750, 20, 5, 16384, 16384);
pid speed_pid3(750, 20, 5, 16384, 16384);
pid speed_pid4(750, 20, 5, 16384, 16384);

pid follow_pid(4.f, 0.f, 0.f, 0.f, 3.f);

chassis_t chassis;

// declaration
void chassis_init();
void chassis_disable();
void chassis_safety_check();
void chassis_data_update();
void chassis_mode_set();
void chassis_reference();
void chassis_update(const float &vx, const float &vy, const float &rotate);
// task
[[noreturn]] void chassis_task(void *args) {
    chassis_init();

    while (true) {
        // 数据更新和离线检测
        chassis_data_update();
        chassis_safety_check();

        if (!chassis.chassis_online) {
            chassis_disable();
        } else {
            chassis_mode_set();
            chassis_reference();

            // control
            chassis_update(chassis.vx_cmd, chassis.vy_cmd, chassis.rotate_cmd);
        }

        vofa::send(E_UART_1, chassis.rc.vx, chassis.rc.vy, chassis.rc.s_l, chassis.yaw_motor_angle, chassis_rx.timestamp);
        os::task::sleep(1);
    }
}

void chassis_init() {
    // motor init
    motor_1.init(), motor_2.init(), motor_3.init(), motor_4.init();
    // msg init
    chassis_rx.init(), chassis_tx.init();
}
void chassis_disable() {
    chassis.vx_cmd = 0, chassis.vy_cmd = 0; chassis.rotate_cmd = 0;
    motor_1.update(0), motor_2.update(0), motor_3.update(0), motor_4.update(0);
    speed_pid1.clear(), speed_pid2.clear(), speed_pid3.clear(), speed_pid4.clear();
}
void chassis_mode_set() {
    switch (chassis.rc.s_l) {
        case -1:chassis.chassis_mode = CHASSIS_DISABLE; break;
        case  0:chassis.chassis_mode = CHASSIS_FOLLOW; break;
        case  1:chassis.chassis_mode = CHASSIS_SPIN; break;
        default: ;
    }
}
void chassis_safety_check() {
    chassis.chassis_online = bsp_time_get_ms() - chassis_rx.timestamp < 100 and chassis_rx.state().data.gimbal_online;
}
void chassis_data_update() {
    auto msg = chassis_rx.state();
    chassis_rx();

    chassis.rc.s_l = msg.data.s_l;
    chassis.rc.vx = msg.data.vx / 220.f;
    chassis.rc.vy = msg.data.vy / 220.f;
    chassis.rc.rotate = msg.data.rotate;

    chassis.yaw_motor_angle = msg.data.yaw_motor_angle;
    chassis.ins_yaw_target = 0;
    chassis.ins_yaw_current = 0;
}
void chassis_reference() {
    switch (chassis.chassis_mode) {
        case CHASSIS_DISABLE : {
            chassis.vx_cmd = 0, chassis.vy_cmd = 0; chassis.rotate_cmd = 0;
            motor_1.update(0), motor_2.update(0), motor_3.update(0), motor_4.update(0);
            speed_pid1.clear(), speed_pid2.clear(), speed_pid3.clear(), speed_pid4.clear();

            break;
            }

        case CHASSIS_FOLLOW : {
            if (chassis.last_chassis_mode != CHASSIS_FOLLOW) {
                follow_pid.clear();
            }
            float relative_yaw = remainder(chassis.yaw_motor_angle, 2.f * fpi);
            // TODO:测试作为前馈值 or pid目标量
            float yaw_error = remainder(chassis.ins_yaw_target - chassis.ins_yaw_current, 2.f * fpi);
            float follow_error = remainder(relative_yaw + yaw_error, 2.f * fpi);

            // 对齐车头或车尾，误差折到(-π/2, π/2]
            while (follow_error > fpi / 2.f) follow_error -= fpi;
            while (follow_error <= -fpi / 2.f) follow_error += fpi;

            chassis.vx_cmd = chassis.rc.vx * cos(relative_yaw) - chassis.rc.vy * sin(relative_yaw);
            chassis.vy_cmd = chassis.rc.vx * sin(relative_yaw) + chassis.rc.vy * cos(relative_yaw);
            chassis.rotate_cmd = follow_pid.update(follow_error, 0.f);

            break;
            }
        case CHASSIS_SPIN : {

            }
        case CHASSIS_SINGLE :
        default: break;
    }

    chassis.last_chassis_mode = chassis.chassis_mode;
}
void chassis_update(const float &vx, const float &vy, const float &rotate) {
    motor_1.update(speed_pid1.update(motor_1.feedback.speed, ((  vy + vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
    motor_2.update(speed_pid2.update(motor_2.feedback.speed, ((  vy - vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
    motor_3.update(speed_pid3.update(motor_3.feedback.speed, (( -vy - vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
    motor_4.update(speed_pid4.update(motor_4.feedback.speed, (( -vy + vx) / SQRT2 - WHEEL_CENTER_DISTANCE * rotate) / WHEEL_RADIUS));
}
