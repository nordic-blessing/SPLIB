/**
  ******************************************************************************
  @file     WS2812B.h
  @brief    内置WS2812B的RGB彩灯驱动：
  @author   Icol Boom < icolboom4@gmail.com >
  @date     2026-01-17 (Created) | 2026-03-08 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-01-17 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_WS2812B_H
#define DEVICE_WS2812B_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "spi.h"

/* Private macros ------------------------------------------------------------*/
#define WS2812_SPI_UNIT     hspi6
#define MAX_RGB             1

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
void WS2812_Send();
void WS2812_Clear();
void WS2812_Set(uint8_t num, uint8_t r, uint8_t g, uint8_t b);
void WS2812_Running(uint8_t r, uint8_t g, uint8_t b, uint8_t tail_len, uint32_t step_ms);
void WS2812_Breath_All(uint8_t base_r, uint8_t base_g, uint8_t base_b, uint32_t step_ms);
void WS2812_Rainbow(uint8_t sat, uint8_t val, uint32_t step_ms);

#endif //DEVICE_WS2812B_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
