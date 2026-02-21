/**
  ******************************************************************************
  @file     bsp_uart.c
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
  ------------------------------------------------------------------------------
  @example
    - 初始化串口中断接收 : 调用`usart_IT_protocol_init()`，开启指定串口的字节中断接收
        `usart_IT_protocol_init()` // 开启已注册串口设备的中断接收
  ------------------------------------------------------------------------------
  @attention
    - 请根据项目更新`uart_IT_protocol_init()`和`HAL_UART_RxCpltCallback()`中的中
        断使能和回调函数
    - 协议解析依赖ProtocolHandler结构体配置（帧头/帧尾长度、包长度、回调函数），需确
        保参数配置正确。如需添加新设备，请参考已有设备的代码。
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
    - 本驱动仅测试了STM32F4/G4系列的部分型号，依赖STM32 HAL库底层初始化
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#include "splib.h"

#if USE_SPLIB_UART

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 串口开始中断接收
 */
void uart_IT_protocol_init() {
#if USE_SPLIB_VOFA_DEBUG
    HAL_UART_Receive_IT(&huart5, &vofa_debug.receive_byte, 1);
#endif
#if USE_SPLIB_VISUAL_UART
    HAL_UART_Receive_IT(&huart4, &visual_uart.receive_byte, 1);
#endif
}

/**
 *
 * @param huart
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
/*
#if USE_SPLIB_ATOMROBOTICS
    uart_RX_decode(huart, &saber_uart);
#endif
#if USE_SPLIB_LASER_L1S
    uart_RX_decode(huart, &Laser_L1s);
#endif
#if USE_SPLIB_REMOTER_SBUS
    uart_RX_decode(huart, &remote_sbus);
#endif
#if USE_SPLIB_VOFA_DEBUG
    uart_RX_decode(huart, &vofa_debug);
#endif
#if USE_SPLIB_VISUAL_UART
    uart_RX_decode(huart, &visual_uart);
#endif
#if USE_SPLIB_WIT_JY_ME01
    uart_RX_decode(huart, &Wit_JY_ME01);
#endif
*/

    if (huart->Instance == UART4) {
#if USE_SPLIB_VISUAL_UART
        uart_RX_decode(huart, &visual_uart);/* 串口设备接收 */
#endif
    } else if (huart->Instance == UART5) {
#if USE_SPLIB_VOFA_DEBUG
        uart_RX_decode(huart, &vofa_debug);/* VOFA 串口调试 */
#endif
    }
}

/**
 * 对带有帧头帧尾的串口协议进行帧头帧尾检测
 * @param huart
 * @param protocolHandler
 */
void uart_RX_decode(UART_HandleTypeDef *huart, ProtocolHandler *protocolHandler) {
    // 溢出检查
    if (!protocolHandler->header_found) {
        if (protocolHandler->buffer_index >= protocolHandler->header_length) {
            protocolHandler->buffer_index = 0;
            return;
        }
    } else {
        uint8_t max_data_len = protocolHandler->package_length - protocolHandler->header_length;
        if (protocolHandler->buffer_index >= max_data_len) {
            protocolHandler->buffer_index = 0;
            protocolHandler->header_found = false;
            return;
        }
    }

    // 帧检测
    if (!protocolHandler->header_found) { // 帧头
        protocolHandler->buffer[protocolHandler->buffer_index] = protocolHandler->receive_byte;
        protocolHandler->buffer_index++;

        if (protocolHandler->buffer_index == protocolHandler->header_length) {
            uint8_t match = 1;
            uint8_t expected = 0, actual = 0;

            for (uint8_t i = 0; i < protocolHandler->header_length; i++) {
                expected = (protocolHandler->header >> ((protocolHandler->header_length - i - 1) * 8)) & 0xFF;
                actual = protocolHandler->buffer[i];
                if (actual != expected) {
                    match = 0;
                    break;
                }
            }

            if (match) {
                protocolHandler->header_found = true;
                protocolHandler->buffer_index = 0;
            } else {
                protocolHandler->buffer_index = 0;
            }
        }
    } else { // 数据域
        protocolHandler->buffer[protocolHandler->header_length +
                                protocolHandler->buffer_index] = protocolHandler->receive_byte;
        protocolHandler->buffer_index++;

        uint8_t match = 1;
        if (protocolHandler->buffer_index == protocolHandler->package_length - protocolHandler->header_length) {
            if (protocolHandler->tail_flag) { // 帧尾
                uint8_t expected = 0, actual = 0;
                uint8_t tail_start = protocolHandler->header_length +
                                     protocolHandler->buffer_index -
                                     protocolHandler->tail_length;

                for (int i = 0; i < protocolHandler->tail_length; i++) {
                    expected = (protocolHandler->tail >> ((protocolHandler->tail_length - 1 - i) * 8)) & 0xFF;
                    actual = protocolHandler->buffer[tail_start + i];
                    if (actual != expected) {
                        match = 0;
                        break;
                    }
                }

            }

            if (match) {
                protocolHandler->callback(protocolHandler->buffer); // 进入回调函数
            }

            protocolHandler->buffer_index = 0;
            protocolHandler->header_found = false;
        }
    }
    HAL_UART_Receive_IT(huart, &protocolHandler->receive_byte, 1);
}

#endif
