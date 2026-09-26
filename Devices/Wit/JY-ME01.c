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
  @example

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
#include "splib.h"

#if USE_SPLIB_WIT_JY_ME01

/* Includes ------------------------------------------------------------------*/
#include "JY-ME01.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float WIT_JY_Yaw;
ProtocolHandler Wit_JY_ME01= {
    .package_length = WIT_ME01_DATA_LENGTH,
    .header = WIT_CMD_R << 8 | Tripod_ID,
    .header_length = WIT_ME01_HEADER_LENGTH,
    .tail_flag = 0,
    .callback = Tripod_Receive,
    
    .buffer_index = 0,
    .header_found = 0
};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

void Tripod_Receive(uint8_t* data) {
    uint32_t temp;
    temp = data[3] << 24 | data[4] << 16 | data[5] << 8 | data[6];

    WIT_JY_Yaw = (float) (temp / 262144.0) * 360.0f;
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
