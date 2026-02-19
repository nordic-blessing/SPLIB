/**
  ******************************************************************************
  @file     bsp_led.h
  @brief    STM32(HAL) LED驱动： 
                - 注册LED句柄列表
                - LED上电自检
                - 获取LED状态
                - 设置LED状态
                - 反转LED状态
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-09-10 (Created) | 2026-02-19 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-19 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef BSP_LED_H
#define BSP_LED_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "gpio.h"

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
typedef enum {
    LED_ON = 0,
    LED_OFF = 1,
    LED_NONE
} LED_State;

typedef struct {
    uint8_t id;
    GPIO_TypeDef *PIN_PORT;
    uint16_t PIN;
    LED_State State;
} LED;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
extern LED led[];
extern uint8_t led_num;

/* Exported function declarations ---------------------------------------------*/
void LEDInit(void);
LED_State GetLEDState(uint8_t led_id);
void SetLEDState(uint8_t led_id, LED_State led_state);
void ToggleLED(uint8_t led_id);

#endif //BSP_LED_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
