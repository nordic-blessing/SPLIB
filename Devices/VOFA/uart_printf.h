//
// Created by Icol_Lee on 2024/11/8.
//

#ifndef USER_USART_PRINTF_H
#define USER_USART_PRINTF_H

#include <stdio.h>
#include <stdarg.h>
#include "usart.h"

#define PRINTF_UART     huart5

void uart_printf(const char *format, ...);

#endif //USER_USART_PRINTF_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
