/**
  ******************************************************************************
  @file     uart_debug.c
  @brief    串口打印调试信息
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2024-11-08 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2024-11-08 [v1.0] Icol Boom: 创建初始版本
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

#if USE_SPLIB_VOFA_PRINTF

/* Includes ------------------------------------------------------------------*/
#include "uart_printf.h"

/* Private define ------------------------------------------------------------*/
#define PRINTF_UART     huart5
#define TX_BUF_SIZE     256

/* Private variables ---------------------------------------------------------*/
uint8_t send_buf[TX_BUF_SIZE];

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 用于串口调试，向电脑发送数据
 * @param format
 * @param ...
 */
void uart_printf(const char *format, ...) {
    va_list args;
    uint32_t length;

    va_start(args, format);
    length = vsnprintf((char *) send_buf, TX_BUF_SIZE, (const char *) format, args);
    va_end(args);

    HAL_UART_Transmit(&PRINTF_UART, (uint8_t *) send_buf, length, 10);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
