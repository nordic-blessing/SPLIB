//
// Created by Icol_Lee on 2026/1/24.
//

#include "splib.h"

#if USE_SPLIB_TJC_UART

#include "tjc_uart_hmi.h"

#define TJC_TX_BUF_SIZE 125

uint8_t tjc_send_buf[TJC_TX_BUF_SIZE];

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

    HAL_UART_Transmit(&TJC_UART, (uint8_t *) tjc_send_buf, length,50);
    uint8_t tail[]={0xff,0xff,0xff};
    HAL_UART_Transmit(&huart5,tail, 3,50);
}

/**
 * 颜色清屏
 * @param color
 */
void tjc_cls(uint16_t color) {
    tjc_printf("cls %d", color);
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
    tjc_printf("xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\"%s\"", posX, posY, posW, posH, 0, fontcolor, backcolor, 1, 1, 1, (const char *) string);
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
    for (int i = 0; i <= w; i++) {
        tjc_printf("line %d,%d,%d,%d,%d", posX1 + i, posY1, posX2 + i, posY2, color);
        tjc_printf("line %d,%d,%d,%d,%d", posX1, posY1 + i, posX2, posY2 + i, color);
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
}

#define grab_a  linkage_a // 大臂长
#define grab_b  linkage_b // 小臂长
void Linkage_hmi(float X, float Y) {
    static float X0_temp, Y0_temp, X1_temp, Y1_temp;

    float l = powf(X, 2) + powf(Y, 2);  //计算末端距离原点的距离平方
    float A1 = acosf((l + powf(grab_a, 2) - powf(grab_b, 2)) / (2 * grab_a * sqrtf(l))) + atanf(Y / X);
    float X0 = X / 2.f;
    float Y0 = Y / 2.f;
    float X1 = grab_a * cosf(A1) / 2.f;
    float Y1 = grab_a * sinf(A1) / 2.f;
    tjc_printf("xstr 60,0,200,50,0,RED,WHITE,1,1,1,\"X:%.1f\"", X);
    tjc_printf("xstr 60,50,200,50,0,RED,WHITE,1,1,1,\"Y:%.1f\"", Y);
    tjc_line(80, 350, 80 + (int) X1_temp, 350 - (int) Y1_temp, 2, 33808);
    tjc_line(80 + (int) X1_temp, 350 - (int) Y1_temp, 80 + (int) X0_temp, 350 - (int) Y0_temp, 1, 33808);
    tjc_cirs(80 + (int) X0_temp, 350 - (int) Y0_temp, 3, 33808);
    tjc_cirs(80 + (int) X0_temp, 350 - (int) Y0_temp + 1, 2, BLUE);
    tjc_line(80, 350, 80 + (int) X1, 350 - (int) Y1, 1, RED);
    tjc_line(80 + (int) X1, 350 - (int) Y1, 80 + (int) X0, 350 - (int) Y0, 1, RED);
    tjc_cirs(80 + (int) X0, 350 - (int) Y0, 3, RED);
    X0_temp = X0, Y0_temp = Y0, X1_temp = X1, Y1_temp = Y1;
}

#endif
