/**
 * @file    HWT906.h
 * @author  Icol_Lee (icolboom4@gmail.com)
 * @brief   
 * @version 0.1
 * @date    2026/5/3
 */
#include "splib.h"

#if USE_SPLIB_WIT_HWT906

#include "HWT906.h"

float WIT_Yaw;

ProtocolHandler wit_uart={
    .package_length =WIT_DATA_LENGTH,
    .header = WIT_HEADER<<8|WIT_TYPE_EULER,
    .header_length = WIT_HEADER_LENGTH,
    .tail_flag = 0,
    .callback = WIT_Receive,

    .buffer_index = 0,
    .header_found = false,
    .buffer = {0}
};


void WIT_Receive(uint8_t* data) {
    uint8_t crc_cal = 0;
    for (uint8_t i = 0; i < WIT_DATA_LENGTH-1; i++) {
        crc_cal += data[i];
    }
    if (crc_cal != data[10]) {
        return;
    }

    WIT_Yaw =(float)(data[7]<<8 | data[6]) / 32768.0f * 180;
    // uart_printf("Yaw: %f\n", WIT_Yaw);
}

#endif
