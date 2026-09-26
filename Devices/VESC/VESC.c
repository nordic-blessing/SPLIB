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
  @example

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
#include "splib.h"

#if USE_SPLIB_VESC

/* Includes ------------------------------------------------------------------*/
#include "VESC.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t vesc_data[4] = {0};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

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

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
