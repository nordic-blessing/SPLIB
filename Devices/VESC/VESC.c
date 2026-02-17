    //
// Created by Icol_Lee on 2025/10/3.
//
#include "splib.h"

#if USE_SPLIB_VESC

#include "VESC.h"

uint8_t vesc_data[4] = {0};

/**
 * 发送控制信息
 * @param mode  值参考VescMode
 * @param id
 * @param value
 */
void vesc_send(enum VescMode mode, uint16_t id, float value) {
    uint32_t Id;
    int32_t data = 0;

    if (mode == kDuty) {
        Id = VESC_CAN_PACKET_SET_DUTY << 8 | id;
        data = (int32_t)(value * 100000);
    } else if (mode == kCurrent) {
        Id = VESC_CAN_PACKET_SET_CURRENT << 8 | id;
        data = (int32_t)(value * 1000);
    } else if (mode == kRpm) {
        Id = VESC_CAN_PACKET_SET_RPM << 8 | id;
        data = (int32_t)(value);
    }

    vesc_data[0] = (data >> 24) & 0xFF;
    vesc_data[1] = (data >> 16) & 0xFF;
    vesc_data[2] = (data >> 8) & 0xFF;
    vesc_data[3] = data & 0xFF;
    CAN_SendExtData(&VESC_CAN, Id, vesc_data, 4);
}

#endif
