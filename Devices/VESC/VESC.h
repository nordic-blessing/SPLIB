/**
  ******************************************************************************
  @file     VESC.c
  @brief    VESC驱动
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-10-03 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2025-10-03 [v1.0] Icol Boom: 创建初始版本
  ------------------------------------------------------------------------------
  @attention
    - 驱动依赖于`bsp_can.c/h`，请务必在`splib_config.h`中使能`USE_SPLIB_CAN`或
        `USE_SPLIB_FDCAN`
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_VESC_H
#define DEVICE_VESC_H

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

#endif //DEVICE_VESC_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
