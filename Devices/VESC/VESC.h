//
// Created by Icol_Lee on 2025/10/3.
//

#ifndef DEVICE_N5065_H
#define DEVICE_N5065_H

#include <stdint.h>

#define VESC_CAN    hfdcan1

#define VESC_CAN_PACKET_SET_DUTY                     0
#define VESC_CAN_PACKET_SET_CURRENT                  1
#define VESC_CAN_PACKET_SET_CURRENT_BRAKE            2
#define VESC_CAN_PACKET_SET_RPM                      3
#define VESC_CAN_PACKET_SET_POS                      4
#define VESC_CAN_PACKET_FILL_RX_BUFFER               5
#define VESC_CAN_PACKET_FILL_RX_BUFFER_LONG          6
#define VESC_CAN_PACKET_PROCESS_RX_BUFFER            7
#define VESC_CAN_PACKET_PROCESS_SHORT_BUFFER         8
#define VESC_CAN_PACKET_STATUS                       9
#define VESC_CAN_PACKET_SET_CURRENT_REL              10
#define VESC_CAN_PACKET_SET_CURRENT_BRAKE_REL        11
#define VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE        12
#define VESC_CAN_PACKET_SET_CURRENT_HANDBRAKE_REL    13
#define VESC_CAN_PACKET_STATUS_2                     14
#define VESC_CAN_PACKET_STATUS_3                     15
#define VESC_CAN_PACKET_STATUS_4                     16
#define VESC_CAN_PACKET_PING                         17
#define VESC_CAN_PACKET_PONG                         18
#define VESC_CAN_PACKET_DETECT_APPLY_ALL_FOC         19
#define VESC_CAN_PACKET_DETECT_APPLY_ALL_FOC_RES     20
#define VESC_CAN_PACKET_CONF_CURRENT_LIMITS          21
#define VESC_CAN_PACKET_CONF_STORE_CURRENT_LIMITS    22
#define VESC_CAN_PACKET_CONF_CURRENT_LIMITS_IN       23
#define VESC_CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN 24
#define VESC_CAN_PACKET_CONF_FOC_ERPMS               25
#define VESC_CAN_PACKET_CONF_STORE_FOC_ERPMS         26
#define VESC_CAN_PACKET_STATUS_5                     27

enum VescMode {
    kDuty = 0,
    kCurrent,
    kRpm
};

void vesc_send(enum VescMode mode, uint16_t id, float value);

#endif //N5065_N5065_H
