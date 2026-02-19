/**
  ******************************************************************************
  @file     bsp_uart.h
  @brief    STM32(HAL) uart私有协议接收驱动： 
                - 串口中断接收初始化（支持多串口设备）
                - 串口中断回调分发
                - 私有协议帧头帧尾检测（带长度校验）及回调触发
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-03-15 (Created) | 2026-02-19 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-19 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

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
void uart_IT_protocol_init(void);
void uart_RX_decode(UART_HandleTypeDef *huart, ProtocolHandler *protocolHandler);

#endif //BSP_UART_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
