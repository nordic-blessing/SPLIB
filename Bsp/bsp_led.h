//
// Created by Icol_Lee on 2025/9/10.
//

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