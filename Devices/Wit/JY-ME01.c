//
// Created by Icol_Lee on 2025/11/14.
//

#include "splib.h"

#if USE_SPLIB_WIT_JY_ME01

#include "JY-ME01.h"

float IMU_Yaw;
ProtocolHandler Wit_JY_ME01= {
        .package_length = WIT_ME01_DATA_LENGTH,
        .header = WIT_CMD_R << 8 | Tripod_ID,
        .header_length = WIT_ME01_HEADER_LENGTH,
        .tail_flag = 0,
        .callback = Tripod_Receive,

        .buffer_index = 0,
        .header_found = 0
};

void Tripod_Receive(uint8_t* data) {
    uint32_t temp;
    temp = data[3] << 24 | data[4] << 16 | data[5] << 8 | data[6];

    IMU_Yaw = (float) (temp / 262144.0) * 360.0f;
}

#endif
