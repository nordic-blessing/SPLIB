/**
  ******************************************************************************
  @file     WS2812B.c
  @brief    内置WS2812B的RGB彩灯驱动：
  @author   Icol Boom < icolboom4@gmail.com >
  @date     2026-01-17 (Created) | 2026-03-08 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-01-17 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ------------------------------------------------------------------------------
  @example
    - 使用示例
        WS2812_Set(0,2,2,2);
        WS2812_Send();
  ------------------------------------------------------------------------------
  @attention
    - 此驱动使用硬件SPI驱动，CubeMX中配置 Data Size 为 8 Bits，速率为 5-10 MBit/s 内，
    CPOL 为 Low，CPHA 为 2 Edge
    - 使用前在 WS2812B.h 中修改 WS2812_SPI_UNIT 宏定义，以及RGB灯数量
    - WS2812_Set()函数仅将指定灯的RGB颜色信息转为WS2812格式，不发送控制命令
    - 本驱动强依赖于STM32 HAL库！
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地让后人知晓
    如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#include "splib.h"

#if USE_SPLIB_WS2812

/* Includes ------------------------------------------------------------------*/
#include "WS2812B.h"

/* Private define ------------------------------------------------------------*/
#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xFC     // 1码

#define BRIGHT_MAX  255 // 最大亮度
#define BRIGHT_STEP 3   // 呼吸变化步长

/* Private variables ---------------------------------------------------------*/
uint8_t WS2812_Data[MAX_RGB][3] = {0};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 将RGB转为WS2812数据格式
 * @param r
 * @param g
 * @param b
 */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++) {
        txbuf[7 - i] = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
}

/**
 * 设置单个RGB灯的颜色（不发送控制命令）
 * @param num
 * @param r
 * @param g
 * @param b
 */
void WS2812_Set(uint8_t num, uint8_t r, uint8_t g, uint8_t b) {
    WS2812_Data[num][0] = (r < 0) ? 0 : ((r > 255) ? 255 : r);
    WS2812_Data[num][1] = (g < 0) ? 0 : ((g > 255) ? 255 : g);
    WS2812_Data[num][2] = (b < 0) ? 0 : ((b > 255) ? 255 : b);
}

/**
 * 发送RGB彩灯控制命令
 */
void WS2812_Send() {
    uint8_t res = 0;

    for (uint8_t i = 0; i < MAX_RGB; i++) {
        WS2812_Ctrl(WS2812_Data[i][0], WS2812_Data[i][1], WS2812_Data[i][2]);
    }

    for (int i = 0; i < 100; i++) {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

/**
 * 清除RGB彩灯颜色信息
 */
void WS2812_Clear() {
    for (uint8_t i = 0; i < MAX_RGB; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            WS2812_Data[i][j] = 0;
        }
    }
}

/**
 * 流水灯
 * @param r
 * @param g
 * @param b
 * @param tail_len  流水灯长度
 * @param step_ms   步长时间
 */
void WS2812_Running(uint8_t r, uint8_t g, uint8_t b, uint8_t tail_len, uint32_t step_ms) {
    static uint8_t head = 0;

    WS2812_Clear();

    for (uint8_t k = 0; k < tail_len; k++) {
        // 计算尾巴位置（向后）
        int16_t idx = (int16_t) head - (int16_t) k;
        if (idx < 0) idx += MAX_RGB;

        // 线性衰减亮度：head最亮，越往后越暗
        uint8_t scale = (uint8_t) ((tail_len - k) * 255 / tail_len);

        uint8_t rr = (uint8_t) ((r * (uint16_t) scale) / 255);
        uint8_t gg = (uint8_t) ((g * (uint16_t) scale) / 255);
        uint8_t bb = (uint8_t) ((b * (uint16_t) scale) / 255);

        WS2812_Set((uint8_t) idx, rr, gg, bb);
    }

    WS2812_Send();

    head++;
    if (head >= MAX_RGB) head = 0;

    DELAY(step_ms);
}

/**
 * 呼吸灯
 * @param base_r
 * @param base_g
 * @param base_b
 * @param step_ms 步长时间
 */
void WS2812_Breath_All(uint8_t base_r, uint8_t base_g, uint8_t base_b, uint32_t step_ms) {
    static int16_t breath = 0;         // 当前亮度 0~BRIGHT_MAX
    static int8_t dir = 1;            // 1: 变亮, -1: 变暗

    // 更新亮度
    breath += dir * BRIGHT_STEP;

    if (breath >= BRIGHT_MAX) {
        breath = BRIGHT_MAX;
        dir = -1;
    } else if (breath <= 20) {
        breath = 20;
        dir = 1;
    }

    // 根据亮度缩放颜色（注意用 uint16_t 防止乘法溢出）
    uint8_t r = (uint8_t) ((base_r * (uint16_t) breath) / BRIGHT_MAX);
    uint8_t g = (uint8_t) ((base_g * (uint16_t) breath) / BRIGHT_MAX);
    uint8_t b = (uint8_t) ((base_b * (uint16_t) breath) / BRIGHT_MAX);

    // 整条灯带都设置成同一个颜色
    for (uint8_t i = 0; i < MAX_RGB; i++) {
        WS2812_Set(i, r, g, b);
    }

    WS2812_Send();

    DELAY(step_ms);
}

/**
 * 将HSV转换为RGB
 * @param h
 * @param s
 * @param v
 * @param r
 * @param g
 * @param b
 */
static void HSV_To_RGB(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (s == 0) {
        // 无饱和度：灰度
        *r = *g = *b = v;
        return;
    }

    uint8_t region = h / 43;              // 0..5
    uint8_t remainder = (h % 43) * 6;     // 0..258

    uint16_t p = (uint16_t)v * (255 - s) / 255;
    uint16_t q = (uint16_t)v * (255 - (uint16_t)s * remainder / 255) / 255;
    uint16_t t = (uint16_t)v * (255 - (uint16_t)s * (255 - remainder) / 255) / 255;

    switch (region) {
        case 0: *r = v;  *g = (uint8_t)t; *b = (uint8_t)p; break;
        case 1: *r = (uint8_t)q; *g = v;  *b = (uint8_t)p; break;
        case 2: *r = (uint8_t)p; *g = v;  *b = (uint8_t)t; break;
        case 3: *r = (uint8_t)p; *g = (uint8_t)q; *b = v;  break;
        case 4: *r = (uint8_t)t; *g = (uint8_t)p; *b = v;  break;
        default:*r = v;  *g = (uint8_t)p; *b = (uint8_t)q; break;
    }
}

/**
 * 炫彩流动
 * @param sat
 * @param val
 * @param step_ms
 */
void WS2812_Rainbow(uint8_t sat, uint8_t val, uint32_t step_ms) {
    static uint8_t baseHue = 0;     // 整体偏移用的 Hue

    if (sat > 255) sat = 255;
    if (val > 255) val = 255;

    // 每颗灯之间的 Hue 间隔
    uint8_t deltaHue = (uint8_t) (256 / MAX_RGB);

    for (uint8_t i = 0; i < MAX_RGB; i++) {
        uint8_t h = baseHue + (uint8_t) (i * deltaHue);

        uint8_t r, g, b;
        HSV_To_RGB(h, sat, val, &r, &g, &b);

        WS2812_Set(i, r, g, b);
    }

    WS2812_Send();

    // 控制“流动速度”：每帧 Hue 偏移 1
    baseHue++;

    DELAY(step_ms);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
