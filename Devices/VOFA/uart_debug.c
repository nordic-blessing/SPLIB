//
// Created by Icol_Lee on 2025/9/22.
//

#include "splib.h"

#if USE_SPLIB_VOFA_DEBUG

#include "uart_debug.h"

Debug_t debugData;
ProtocolHandler vofa_debug={
        .package_length = DEBUG_DATA_LENGTH,
        .header = DEBUG_HEADER,
        .header_length = DEBUG_HEADER_LENGTH,
        .tail_flag = 1,
        .tail = DEBUG_TAIL,
        .tail_length = DEBUG_TAIL_LENGTH,
        .callback = Debug_Receive,

        .buffer_index = 0,
        .header_found = false,
        .buffer = {0}
};

void Debug_Receive(uint8_t* data) {
    uint32_t temp;
    float floatValue;

    temp = data[5] << 24 | data[4] << 16 | data[3] << 8 | data[2];
    memcpy(&floatValue, &temp, sizeof(float));

    switch (data[1]) {
        case 0xAA:
            debugData.start = floatValue;
            break;
        case 0x01:
            debugData.data1 = floatValue;
            break;
        case 0x02:
            debugData.data2 = floatValue;
            break;
        case 0x03:
            debugData.data3 = floatValue;
            break;
        case 0x04:
            debugData.data4 = floatValue;
            break;
        case 0x05:
            debugData.data5 = floatValue;
            break;
        case 0x06:
            debugData.data6 = floatValue;
            break;
        default:
            break;
    }
//    uart_printf("%f\r\n",floatValue);
}

#endif
