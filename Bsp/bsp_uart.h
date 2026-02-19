//
// Created by Icol_Lee on 2025/9/21.
//

#ifndef BSP_UART_H
#define BSP_UART_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "usart.h"

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t package_length;         //数据长度
    uint16_t header;                //帧头
    uint8_t header_length;          //帧头长度
    bool tail_flag;                 //帧尾标志位
    uint16_t tail;                  //帧尾
    uint8_t tail_length;            //帧尾长度
    void (*callback)(uint8_t*);     //回调函数

    uint8_t buffer_index;       //当前接收区索引
    bool header_found;          //是否找到帧头
    uint8_t receive_byte;       //接收字节
    uint8_t buffer[256];        //数据接收区
} ProtocolHandler;

/* Exported variables ---------------------------------------------------------*/
#if USE_SPLIB_LASER_L1S
    extern ProtocolHandler Laser_L1s;
#endif
#if USE_SPLIB_REMOTER_SBUS
    extern ProtocolHandler remote_sbus;
#endif
#if USE_SPLIB_VOFA_DEBUG
    extern ProtocolHandler vofa_debug;
#endif
#if USE_SPLIB_VISUAL_UART
    extern ProtocolHandler visual_uart;
#endif
#if USE_SPLIB_WIT_JY_ME01
    extern ProtocolHandler Wit_JY_ME01;
#endif

/* Exported function declarations ---------------------------------------------*/
void usart_IT_protocol_init(void);
void uart_RX_decode(UART_HandleTypeDef *huart, ProtocolHandler *protocolHandler);


#endif //BSP_UART_H
