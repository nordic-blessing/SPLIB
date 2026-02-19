//
// Created by Icol_Lee on 2025/10/17.
//

#include "splib.h"

#if USE_SPLIB_LASER_L1S

#include "Laser_L1s.h"

uint32_t Laser_distance;
ProtocolHandler Laser_L1s = {
        .package_length = Laser_L1s_Rx_DATA_LENGTH,
        .header = Laser_L1s_Rx_HEADER1 << 8 | Laser_L1s_Rx_HEADER2,
        .header_length = Laser_L1s_Rx_HEADER_LENGTH,
        .tail_flag = 0,
        .callback = Laser_L1s_Receive,

        .buffer_index = 0,
        .header_found = false,
        .buffer = {0}
};

void Laser_L1s_Receive(uint8_t *data) {
    uint32_t temp;
    uint8_t error;

    temp = data[3] << 24 | data[4] << 16 | data[5] << 8 | data[6];
    if (!(data[2] & 0x80)) {
        Laser_distance = (temp);
    }else{
        error = temp;
    }
}

#endif
