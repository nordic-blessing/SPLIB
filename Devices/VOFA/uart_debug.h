/**
  ******************************************************************************
  @file     uart_debug.c
  @brief    私有vofa通信协议，用于上位机调试
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-09-22 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2025-09-22 [v1.0] Icol Boom: 创建初始版本
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

#ifndef DEVICE_UART_DEBUG_H
#define DEVICE_UART_DEBUG_H

#include <stdint.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define DEBUG_HEADER        0xDF
#define DEBUG_HEADER_LENGTH 1u
#define DEBUG_TAIL          0xFF
#define DEBUG_TAIL_LENGTH   1u
#define DEBUG_DATA_LENGTH   7u

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
    float start;
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
}Debug_t;

/* Exported variables ---------------------------------------------------------*/
extern Debug_t debugData;

/* Exported function declarations ---------------------------------------------*/
void Debug_Receive(uint8_t* data);

#endif //DEVICE_UART_DEBUG_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
