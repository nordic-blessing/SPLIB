/**
  ******************************************************************************
  @file     uart_debug.c
  @brief    私有通信协议，用于接收上位机信息
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-09-22 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2025-09-22 [v1.0] Icol Boom: 创建初始版本
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

#if USE_SPLIB_VOFA_DEBUG

/* Includes ------------------------------------------------------------------*/
#include "uart_debug.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
Debug_t debugData;
ProtocolHandler vofa_debug={
        .package_length = DEBUG_DATA_LENGTH,
        .header = DEBUG_HEADER,
        .header_length = DEBUG_HEADER_LENGTH,
        .tail_flag = 1,
        .tail = DEBUG_TAIL,
        .tail_length = DEBUG_TAIL_LENGTH,
        .callback = Debug_Receive,

        .buffer_index = 0,
        .header_found = false,
        .buffer = {0}
};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

void Debug_Receive(uint8_t* data) {
    uint32_t temp;
    float floatValue;

    temp = data[5] << 24 | data[4] << 16 | data[3] << 8 | data[2];
    memcpy(&floatValue, &temp, sizeof(float));

    switch (data[1]) {
        case 0xAA:
            debugData.start = floatValue;
            break;
        case 0x01:
            debugData.data1 = floatValue;
            break;
        case 0x02:
            debugData.data2 = floatValue;
            break;
        case 0x03:
            debugData.data3 = floatValue;
            break;
        case 0x04:
            debugData.data4 = floatValue;
            break;
        case 0x05:
            debugData.data5 = floatValue;
            break;
        case 0x06:
            debugData.data6 = floatValue;
            break;
        default:
            break;
    }
//    uart_printf("%f\r\n",floatValue);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
