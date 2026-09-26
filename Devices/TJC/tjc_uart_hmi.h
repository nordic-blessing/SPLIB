/**
  ******************************************************************************
  @file     tjc_uart_hmi.c
  @brief    TJC串口屏下位机绘制：
                - 纯色清屏
                - 矩形绘制
                - 带背景颜色的字符串绘制
                - 线条绘制
                - 圆形轮廓绘制
                - 插图图片
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-01-24 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2025-01-24 [v1.0] Icol Boom: 创建初始版本
  ------------------------------------------------------------------------------
  @example
    - 纯色清屏 : 使用单一颜色覆盖全屏
        `tjc_cls(RED)` // 红色清屏
    - 矩形画图
        `tjc_fill(0, 0, 10, 20, RED)` // 以（0，0）为起点画一个长10高20的矩形
    - 绘制字符串
        `tjc_xstr(0, 0, 30, 10, BLACK, WHITE, "CH")` // 在以（0,0）为起点，长30高10的白色矩形内，填入黑色字符串“CH”
    - 线条绘制
        `tjc_line(0, 0, 10, 50, 2, GREEN)` // 起点（0,0），终点（10,50），粗为2的绿色直线
    - 圆形轮廓绘制
        `tjc_cirs(10, 10, 8, YELLOW)` // 圆心为（10,10），半径为8
    - 插入图片
        `tjc_pic(10, 5, 1)` //以（10,5）为起点，插入id为1的图片
  ------------------------------------------------------------------------------
  @attention
    - 驱动依赖于串口外设，需要正确配置串口
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_TJC_UART_HMI_H
#define DEVICE_TJC_UART_HMI_H

#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
//#include "usart.h"

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
void tjc_pic(int posX, int posY, int id);
void Linkage_hmi(float X, float Y);

#endif //DEVICE_TJC_UART_HMI_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
