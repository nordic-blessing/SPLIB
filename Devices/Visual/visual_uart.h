/**
  ******************************************************************************
  @file     visual_uart.c
  @brief    私有串口协议驱动，与视觉组通信
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2026-06-02 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-06-02 [v1.0] Icol Boom: 创建初始版本
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

#ifndef DEVICE_VISUAL_H
#define DEVICE_VISUAL_H

#include <stdint.h>
#include <string.h>

#define VISUAL_HEADER1        0xAA
#define VISUAL_HEADER2        0x55
#define VISUAL_HEADER_LENGTH    1u
#define VISUAL_TAIL1          0x0F
#define VISUAL_TAIL2          0x0A
#define VISUAL_TAIL_LENGTH      1u
#define VISUAL_DATA_LENGTH      28

typedef struct {
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
}Visual_t;

extern Visual_t visualData;

void Visual_Receive(uint8_t *data);

#endif //DEVICE_VISUAL_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
