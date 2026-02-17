//
// Created by Icol_Lee on 2025/9/20.
//

#include "splib.h"

#if USE_SPLIB_DJI

#include "M3508.h"

uint8_t m3508_data[2][8] = {0};

/**
 * 电机结构体初始化
 * @param ptr
 * @param id
 * @param hfdcan
 */
void m3508_init(DJI_t *ptr, uint8_t id, FDCAN_HandleTypeDef *hfdcan) {
    ptr->can_handle = hfdcan;

    ptr->id = id;
    ptr->id_group = id > 4 ? 1 : 0;

    ptr->pos = 0;
    ptr->speed_rpm = 0;
    ptr->real_current = 0.0f;
    ptr->temp = 0;

    ptr->offset_flag = 0;
    ptr->offset_pos = 0;
    ptr->last_pos = 0;
    ptr->round_cnt = 0;
    ptr->all_pos = 0;
    ptr->angle = 0.0f;

    ptr->control_row = ptr->id_group;
    ptr->controlH_col = 2 * ((ptr->id + ptr->id_group) % 5) - 2;
    ptr->controlL_col = 2 * ((ptr->id + ptr->id_group) % 5) - 1;
}

/**
 * 接收电调反馈信息，此函数应该放置在接收中断中
 * @param ptr
 * @param rx    从中断中接受到的数据
 */
void m3508_receive(DJI_t *ptr, uint8_t rx[]) {
    ptr->last_pos = ptr->pos;
    ptr->pos = (uint16_t) (rx[0] << 8 | rx[1]);
    ptr->speed_rpm = (int16_t) (rx[2] << 8 | rx[3]) / 19;
    ptr->real_current = (float) (rx[4] << 8 | rx[5]) * 5.f / 16384.f;
    ptr->temp = rx[6];

    if ((float) (ptr->pos - ptr->last_pos) > 4096.0f)
        ptr->round_cnt--;
    else if ((float) (ptr->pos - ptr->last_pos) < -4096.0f)
        ptr->round_cnt++;

    //记录初始位置
    if (!ptr->offset_flag) {
        ptr->offset_pos = ptr->pos;
        ptr->round_cnt = 0;
        ptr->offset_flag = 1;
    }

    ptr->all_pos = ptr->round_cnt * 8192 + ptr->pos - ptr->offset_pos;
    ptr->angle = ((float) ptr->all_pos / 8192.0f) * 360.0f;
}

/**
 * 发送控制信息
 * @param ptr
 * @param iq 控制电流值
 */
#define ABS(x)  x > 0 ? x : (- x)
void m3508_send(DJI_t *ptr, int16_t iq) {
    //控制电流范围 [-16384, 16384]
    if (ABS(iq) < 16384) {
        ptr->given_current = iq;
        m3508_data[ptr->control_row][ptr->controlH_col] = iq >> 8;
        m3508_data[ptr->control_row][ptr->controlL_col] = iq & 0xFF;
        CAN_SendStdData(ptr->can_handle, ptr->id_group ? 0x1FF : 0x200, m3508_data[ptr->control_row],
                        FDCAN_DLC_BYTES_8);
    }
}

#endif
