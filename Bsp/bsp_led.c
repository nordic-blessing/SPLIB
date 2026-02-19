/**
  ******************************************************************************
  @file     bsp_led.c
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
  ------------------------------------------------------------------------------
  @example
    - 初始化LED : 调用`LEDInit()`, 完成LED上电自检
        `LEDInit()` // LED依次点亮，循环闪烁2次后全部熄灭
    - 获取LED状态 : 调用`GetLEDState()`, 获取指定ID的LED当前状态
        `LED_State state = GetLEDState(0)` // 获取 ID=0 状态，返回LED_ON/LED_OFF/LED_NONE
    - 设置LED状态 : 调用`SetLEDState()`, 设置指定ID的LED为点亮/熄灭状态
        `SetLEDState(1, LED_ON)` // 将 ID=1 设置为点亮状态
    - 反转LED状态 : 调用`ToggleLED()`, 反转指定ID的LED当前状态
        `ToggleLED(0)` // 反转 ID=0 状态
  ------------------------------------------------------------------------------
  @attention
    - 使用前请在`splib_config.h`中使能`USE_SPLIB_LED`
    - 请根据项目具体情况更新`led[]`列表
    - `LEDInit()`函数常用来用在系统启动的第一步，等待所有接入的设备彻底启动
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
    - 本驱动仅测试了STM32F4/G4系列的部分型号，依赖STM32 HAL库底层初始化
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#include "splib.h"

#if USE_SPLIB_LED

/* Includes ------------------------------------------------------------------*/
#include "bsp_led.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LED led[] = {
        {0, LED1_GPIO_Port, LED1_Pin, LED_OFF},
        {1, LED2_GPIO_Port, LED2_Pin, LED_OFF}
};
uint8_t led_num = sizeof(led) / sizeof(led[0]);

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 初始化LED状态
 */
void LEDInit(void) {
    for (uint8_t i = 0; i < led_num; i++) {
        SetLEDState(i, LED_ON);
        DELAY(140);
    }

    for (int j = 0; j < 5; j++) {
        for (uint8_t i = 0; i < led_num; i++) {
            ToggleLED(i);
            DELAY(70);
        }
    }

    DELAY(300);

    for (int j = 0; j < 5; j++) {
        for (uint8_t i = 0; i < led_num; i++) {
            ToggleLED(i);
        }
        DELAY(140);
    }

    for (uint8_t i = 0; i < led_num; i++) {
        SetLEDState(i, LED_OFF);
    }
}

/**
 * 读取LED状态
 */
LED_State GetLEDState(uint8_t led_id) {
    if (led_id <= led_num - 1)
        return led[led_id].State ? 0 : 1;
    else
        return LED_NONE;
}

/**
 * 设置LED状态
 */
void SetLEDState(uint8_t led_id, LED_State led_state) {
    HAL_GPIO_WritePin(led[led_id].PIN_PORT, led[led_id].PIN, led_state);
    led[led_id].State = led_state;
}

/**
 * 反转LED状态
 */
void ToggleLED(uint8_t led_id) {
    HAL_GPIO_TogglePin(led[led_id].PIN_PORT, led[led_id].PIN);
    led[led_id].State ^= 1;
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
