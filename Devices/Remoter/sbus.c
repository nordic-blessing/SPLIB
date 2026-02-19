//
// Created by 30958 on 2026/1/14.
//

#include "splib.h"

#if USE_SPLIB_REMOTER_SBUS

#include "sbus.h"

Remoter_sbus RemoterData;
ProtocolHandler remote_sbus={
        .package_length = REMOTER_DATA_LENGTH,
        .header = REMOTER_HEADER,
        .header_length = REMOTER_HEADER_LENGTH,
        .tail_flag = 1,
        .tail = REMOTER_TAIL,
        .tail_length = REMOTER_TAIL_LENGTH,
        .callback = Remoter_Receive,

        .buffer_index = 0,
        .header_found = false,
        .buffer = {0}
};

void Remoter_Receive(uint8_t* data) {
    if (data == NULL) {
        return;
    }
    uint16_t temp;

    temp = ((data[1] | data[2] << 8) & 0x07ff);
    RemoterData.CH1 = ((float)temp - 992.f)/8.f;
    temp = (data[2] >> 3 | data[3] << 5) & 0x07ff;
    RemoterData.CH2 = ((float)temp - 992.f)/8.f;
    temp = (data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07ff;
    RemoterData.CH3 = ((float)temp - 992.f)/8.f;
    temp = (data[5] >> 1 | data[6] << 7) & 0x07ff;
    RemoterData.CH4 = ((float)temp - 992.f)/8.f;
    temp = (data[6] >> 4 | data[7] << 4) & 0x07ff;
    RemoterData.CH5 = ((float)temp - 992.f)/8.f;
    temp = (data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07ff;
    RemoterData.CH6 = ((float)temp - 992.f)/8.f;

//    usart5_printf("%f,%f,%f,%f,%f,%f\r\n", RemoterData.CH1, RemoterData.CH2, RemoterData.CH3,RemoterData.CH4,RemoterData.CH5,RemoterData.CH6);
}

#endif
