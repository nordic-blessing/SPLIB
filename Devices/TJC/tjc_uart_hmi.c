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

#include "splib.h"

#if USE_SPLIB_TJC_UART

/* Includes ------------------------------------------------------------------*/
#include "tjc_uart_hmi.h"

/* Private define ------------------------------------------------------------*/
#define TJC_UART    huart3
#define TJC_TX_BUF_SIZE 125
#define TJC_SEND(data, length)  HAL_UART_Transmit(&TJC_UART, data, length, 50)
/* Private variables ---------------------------------------------------------*/
uint8_t tjc_send_buf[TJC_TX_BUF_SIZE];

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 用于陶晶驰串口屏的指令发送
 * @param format
 * @param ...
 */
void tjc_printf(const char *format, ...) {
    va_list args;
    uint32_t length;

    va_start(args, format);
    length = vsnprintf((char *) tjc_send_buf, TJC_TX_BUF_SIZE, (const char *) format, args);
    va_end(args);

    TJC_SEND((uint8_t *)tjc_send_buf, length);
}

/**
 * 颜色清屏
 * @param color
 */
void tjc_cls(uint16_t color) {
    tjc_printf("cls %d", color);
    uint8_t tail[]={0xff,0xff,0xff};
    TJC_SEND(tail, 3);
}

/**
 * 矩形颜色填充
 * @param posX
 * @param posY
 * @param posW
 * @param posH
 * @param color
 */
void tjc_fill(int posX, int posY, int posW,  int posH, int color) {
    tjc_printf("fill %d,%d,%d,%d,%d", posX, posY, posW, posH, color);
    uint8_t tail[]={0xff,0xff,0xff};
    TJC_SEND(tail, 3);
}

/**
 * 插入字符串
 * @param posX
 * @param posY
 * @param posW
 * @param posH
 * @param fontcolor
 * @param backcolor
 * @param string
 */
void tjc_xstr(int posX, int posY, int posW, int posH, int fontcolor, int backcolor, const char *string) {
    tjc_printf("xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,", posX, posY, posW, posH, 0, fontcolor, backcolor, 1, 1, 1);
    tjc_printf("\"%s\"", (const char *) string);
    uint8_t tail[]={0xff,0xff,0xff};
    TJC_SEND(tail, 3);
}

/**
 * 画线
 * @param posX1
 * @param posY1
 * @param posX2
 * @param posY2
 * @param w
 * @param color
 */
void tjc_line(int posX1, int posY1, int posX2, int posY2, int w, int color) {
    tjc_printf("line %d,%d,%d,%d,%d", posX1, posY1, posX2, posY2, color);
    uint8_t tail[]={0xff,0xff,0xff};
    TJC_SEND(tail, 3);
    for (int i = 0; i <= w; i++) {
        tjc_printf("line %d,%d,%d,%d,%d", posX1 + i, posY1, posX2 + i, posY2, color);
        TJC_SEND(tail, 3);
        tjc_printf("line %d,%d,%d,%d,%d", posX1, posY1 + i, posX2, posY2 + i, color);
        TJC_SEND(tail, 3);
    }
}

/**
 * 画圆
 * @param posX
 * @param posY
 * @param r
 * @param color
 */
void tjc_cirs(int posX, int posY, int r, int color) {
    tjc_printf("cirs %d,%d,%d,%d", posX, posY, r, color);
    uint8_t tail[]={0xff,0xff,0xff};
    TJC_SEND(tail, 3);
}

void tjc_pic(int posX, int posY, int id) {
    tjc_printf("pic %d,%d,%d", posX, posY, id);
    uint8_t tail[]={0xff,0xff,0xff};
    TJC_SEND(tail, 3);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
