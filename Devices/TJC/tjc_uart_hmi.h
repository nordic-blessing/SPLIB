//
// Created by Icol_Lee on 2026/1/24.
//

#ifndef DEVICE_TJC_UART_HMI_H
#define DEVICE_TJC_UART_HMI_H

#include <stdarg.h>
#include <stdio.h>
#include "usart.h"

#define TJC_UART    huart3

// HMI颜色
#define RED     63488
#define BLUE    31
#define GRAY    33840
#define BLACK   0
#define WHITE   65535
#define GREEN   2016
#define BROWN   48192
#define YELLOW  65504

void tjc_printf(const char *format, ...);
void tjc_cls(uint16_t color);
void tjc_fill(int posX, int posY, int posW,  int posH, int color);
void tjc_xstr(int posX, int posY, int posW, int posH, int fontcolor, int backcolor, const char *string);
void tjc_line(int posX1, int posY1, int posX2, int posY2, int w, int color);
void tjc_cirs(int posX, int posY, int r, int color);
void Linkage_hmi(float X, float Y);

#endif //DEVICE_TJC_UART_HMI_H
