//
// Created by 30958 on 2025/9/10.
//
#include "splib.h"

#if USE_SPLIB_LED

#include "bsp_led.h"

LED led[] = {
        {0, LED1_GPIO_Port, LED1_Pin, LED_OFF},
        {1, LED2_GPIO_Port, LED2_Pin, LED_OFF}
};
uint8_t led_num = sizeof(led) / sizeof(led[0]);

/**
 * 初始化LED状态
 */
void LEDInit(void) {
    for (uint8_t i = 0; i < led_num; i++) {
        SetLEDState(i, LED_ON);
        DELAY(80);
    }

    for (int j = 0; j < 2; j++) {
        for (uint8_t i = 0; i < led_num; i++) {
            ToggleLED(i);
            DELAY(80);
        }
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
