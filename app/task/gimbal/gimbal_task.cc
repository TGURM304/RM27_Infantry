//
// Created by double_J on 2026/10/31.
//

#include "def.h"
#include "gimbal_task.h"
#include "SMC/slidingmodec.h"
#include "controller/pid.h"
#include "utils/os.h"
#include "motor/dm.h"
#include "utils/msg.h"
#include "rc/dr16.h"
#include "utils/vofa.h"
#include "ins/ins.h"

constexpr float fpi = M_PI;
constexpr float SQRT2 = M_SQRT2;
constexpr float g = 9.80665f;

using namespace motor;
using namespace controller;
// DM 4340 位置速度模式
dm fold_pitch_motor("Fold", {.slave_id = 0x03,.master_id = 0x23,.port = E_CAN_2,.mode = dm::POSITION_SPEED,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});
// DM 4310 MIT 模式
dm pitch_motor("pitch", {.slave_id = 0x05,.master_id = 0x25,.port = E_CAN_2,.mode = dm::MIT,.p_max = 12.5f,.v_max = 30.0f,.t_max = 10.0f});
// DM 4310 MIT 模式
dm yaw_motor("Yaw", {.slave_id = 0x04,.master_id = 0x24,.port = E_CAN_3,.mode = dm::MIT,.p_max = 3.1415926535f,.v_max = 30.0f,.t_max = 10.0f});

// 滑膜控制算法
smc YawSMC(30.f, 50.f, 0.001f, 21.f, 27.f, 4.f, 0.00035f, 0.5f);

//PID
pid pitch_speed(1.5, 0.005, 0, 1, 3);
pid pitch_angle(22, 0, 0.0, 0, 20);


msg::can_sender<data_to_chassis, E_CAN_3, 0x311> gimbal_tx;
msg::can_receiver<data_to_gimbal, E_CAN_3, 0x322> gimbal_rx;

gimbal_t gimbal;
// declaration
void gimbal_init();
void gimbal_safety_check();
void gimbal_mode_set();
void gimbal_data_update();
void gimbal_reference();
void gimbal_update();
// task
[[noreturn]] void gimbal_task(void *args) {
    gimbal_init();
    uint8_t cnt = 0;
    while (true) {
        // 数据更新和离线检测
        gimbal_data_update();
        gimbal_safety_check();

        if (!gimbal.gimbal_online) {
            gimbal.last_gimbal_mode = GIMBAL_DISABLE;
            gimbal.yaw_torque = 0.f;
        } else {
            if (++cnt > 10) {
                auto msg = gimbal_tx();
                msg->s_l = gimbal.rc.s_l;
                msg->gimbal_online = gimbal.gimbal_online;
                msg->vx = gimbal.rc.rc_l[0];
                msg->vy = gimbal.rc.rc_l[1];
                msg->rotate = gimbal.rc.reserved;
                msg->yaw_motor_angle = yaw_motor.feedback.pos;
                msg->yaw_rc_delta = -gimbal.yaw_rc_delta;
                gimbal_tx.send();
                cnt = 0;
            }
            gimbal_mode_set();
            gimbal_reference();
            gimbal_update();
        }


        // vofa::send(E_UART_1, pitch_motor.feedback.pos, pitch_motor.feedback.vel, gimbal.ins.pitch, gimbal.ins.pitch_vel, pitch_motor.feedback.vel, gimbal.pitch_torque);
        // vofa::send(E_UART_1, fold_pitch_motor.feedback.vel);
        // vofa::send(E_UART_1, remainder(yaw_motor.feedback.pos, 2.f * fpi));
        vofa::send(E_UART_1, fold_pitch_motor.feedback.pos, fold_pitch_motor.feedback.temp_rotor, fold_pitch_motor.feedback.temp_mos);
        os::task::sleep(1);
    }
}

void gimbal_init() {
    // motor init
    fold_pitch_motor.init(), pitch_motor.init(), yaw_motor.init();
    fold_pitch_motor.reset(), pitch_motor.reset(), yaw_motor.reset();
    fold_pitch_motor.enable(), pitch_motor.enable(), yaw_motor.enable();
    // msg init
    gimbal_tx.init(), gimbal_rx.init();
    // variables init
    gimbal.yaw_target = ins::data()->yaw * 180.f / fpi;
    gimbal.yaw_current= ins::data()->yaw * 180.f / fpi;
    
    gimbal.gimbal_online = false;
}
void gimbal_safety_check() {
    gimbal.gimbal_online = gimbal.rc.timestamp != 0 && bsp_time_get_ms() - gimbal.rc.timestamp < 100;

    if (!gimbal.gimbal_online) {
        fold_pitch_motor.disable(), pitch_motor.disable(), yaw_motor.disable();
    } else {
        // fold_pitch_motor.reset(), pitch_motor.reset(), yaw_motor.reset();
        fold_pitch_motor.enable(), pitch_motor.enable(), yaw_motor.enable();
    }
}
void gimbal_mode_set() {
    if (gimbal.rc.s_l != -1) {
        gimbal.gimbal_mode = GIMBAL_FOLD;
        if (gimbal.rc.s_r != -1) gimbal.gimbal_mode = GIMBAL_UNFOLD;
    }
    else gimbal.gimbal_mode = GIMBAL_DISABLE;

}
void gimbal_data_update() {
    gimbal.rc = rc::dr16::state();
    gimbal.ins.pitch = -((ins::data()->roll > 0.0f) ? (ins::data()->roll - fpi) : (ins::data()->roll + fpi));
    gimbal.ins.yaw = ins::data()->yaw * 180.f / fpi;
    gimbal.ins.yaw_vel = -ins::data()->gyro[2] * 180.f / fpi;

    gimbal.ins.pitch_vel = -ins::data()->gyro[0];
}
void gimbal_reference() {
    gimbal.yaw_torque = 0.f;
    switch (gimbal.gimbal_mode) {
        case GIMBAL_DISABLE: {
            fold_pitch_motor.disable();
            // gimbal.fold_motor_target_angle = FOLD_MOTOR_DOWN_LIMIT;
            gimbal.pitch_torque = 0.f;
            gimbal.yaw_torque = 0.f;
            break;
            }

        case GIMBAL_FOLD : {
            if (gimbal.last_gimbal_mode != GIMBAL_FOLD) {
                gimbal.yaw_target = gimbal.ins.yaw;
            }
            //fold pitch
            gimbal.fold_motor_target_angle = FOLD_MOTOR_DOWN_LIMIT;
            // yaw
            gimbal.yaw_rc_delta = static_cast<float>(gimbal.rc.rc_r[0]) * 0.5f / 660.f;
            gimbal.yaw_target = remainder(gimbal.yaw_target - gimbal.yaw_rc_delta, 360.f);
            gimbal.yaw_torque = YawSMC.smc_update(gimbal.yaw_target, gimbal.ins.yaw, gimbal.ins.yaw_vel);
            // pitch
            gimbal.pitch_target = 0.f;
            float pitch_spd = pitch_angle.update(gimbal.ins.pitch, 0.f);
            float pitch_feedforward = -GIMBAL_MASS * g * cosf(-gimbal.ins.pitch - 3.67f * fpi / 180.f) * 0.033;
            gimbal.pitch_torque = pitch_feedforward + pitch_speed.update(pitch_motor.feedback.vel, pitch_spd);

            // gimbal.pitch_torque = pitch_feedforward;
            // vofa::send(E_UART_1, pitch_feedforward, gimbal.ins.pitch);
            break;
        }
        case GIMBAL_UNFOLD : {
            if (gimbal.last_gimbal_mode != GIMBAL_UNFOLD) {
                gimbal.yaw_target = gimbal.ins.yaw;
            }
            //fold pitch
            gimbal.fold_motor_target_angle = FOLD_MOTOR_UP_LIMIT;
            // yaw
            gimbal.yaw_rc_delta = static_cast<float>(gimbal.rc.rc_r[0]) * 0.5f / 660.f;
            gimbal.yaw_target = remainder(gimbal.yaw_target - gimbal.yaw_rc_delta, 360.f);
            gimbal.yaw_torque = YawSMC.smc_update(gimbal.yaw_target, gimbal.ins.yaw, gimbal.ins.yaw_vel);
            // pitch
            gimbal.pitch_target = 0.f;
            float pitch_spd = pitch_angle.update(gimbal.ins.pitch, 0.f);
            float pitch_feedforward = -GIMBAL_MASS * g * cosf(-gimbal.ins.pitch - 3.67f * fpi / 180.f) * 0.033;
            gimbal.pitch_torque = pitch_feedforward + pitch_speed.update(pitch_motor.feedback.vel, pitch_spd);

            break;
        }
        case GIMBAL_AUTO_AIM:
        default: break;
    }

    gimbal.last_gimbal_mode = gimbal.gimbal_mode;
}

void gimbal_update() {
    fold_pitch_motor.control(gimbal.fold_motor_target_angle, 28);
    pitch_motor.control(0.f, 0.f, 0.f, 0.f, gimbal.pitch_torque);
    yaw_motor.control(0.f, 0.f, 0.f, 0.f, gimbal.yaw_torque);
}
