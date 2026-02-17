//
// Created by Icol_Lee on 2025/9/21.
//

#include "splib.h"

#if USE_SPLIB_UART

#include "bsp_uart.h"

/**
 * 串口开始中断接收
 */
void usart_IT_protocol_init() {
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
#if USE_SPLIB_LASER_L1S
    uart_RX_decode(huart, Laser_L1s);
#endif
#if USE_SPLIB_REMOTER_SBUS
    uart_RX_decode(huart, remote_sbus);
#endif
#if USE_SPLIB_VOFA_DEBUG
    uart_RX_decode(huart, vofa_debug);
#endif
#if USE_SPLIB_VISUAL_UART
    uart_RX_decode(huart, visual_uart);
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
