//
// Created by Icol_Lee on 2025/9/22.
//

#ifndef DEVICE_UART_DEBUG_H
#define DEVICE_UART_DEBUG_H

#include <stdint.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define DEBUG_HEADER        0xDF
#define DEBUG_HEADER_LENGTH 1u
#define DEBUG_TAIL          0xFF
#define DEBUG_TAIL_LENGTH   1u
#define DEBUG_DATA_LENGTH   7u

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
    float start;
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
}Debug_t;

/* Exported variables ---------------------------------------------------------*/
extern Debug_t debugData;

/* Exported function declarations ---------------------------------------------*/
void Debug_Receive(uint8_t* data);

#endif //DEVICE_UART_DEBUG_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
