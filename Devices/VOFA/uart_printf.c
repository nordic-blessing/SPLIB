//
// Created by Icol_Lee on 2024/11/8.
//
#include "splib.h"

#if USE_SPLIB_VOFA_PRINTF

/* Includes ------------------------------------------------------------------*/
#include "uart_printf.h"

/* Private define ------------------------------------------------------------*/
#define TX_BUF_SIZE 256

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

    HAL_UART_Transmit(&PRINTF_UART, (uint8_t *) send_buf, length, 50);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
