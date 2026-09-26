/**
  ******************************************************************************
  @file     JY_ME01.c
  @brief    维特智能IMU驱动
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-11-14 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2025-11-14 [v1.0] Icol Boom: 创建初始版本
  ------------------------------------------------------------------------------
  @attention
    - 驱动依赖于`bsp_uart.c/h`，请务必在`splib_config.h`中使能`USE_SPLIB_UART`
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#ifndef DEVICE_WIT_JY_ME01_H
#define DEVICE_WIT_JY_ME01_H

#include <stdint.h>

// ID
#define Tripod_ID               0x01

// CMD
#define WIT_CMD_R               0x03
#define WIT_CMD_W               0x06

// Length
#define WIT_ME01_HEADER_LENGTH  2u
#define WIT_ME01_DATA_LENGTH    9u

extern float WIT_JY_Yaw;

void Tripod_Receive(uint8_t* data);

#endif //DEVICE_WIT_JY_ME01_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
