//
// Created by Icol_Lee on 2025/11/14.
//

#ifndef DEVICE_WIT_JY_ME01_H
#define DEVICE_WIT_JY_ME01_H

#include <stdint.h>

// ID
#define Tripod_ID               0x01

// CMD
#define WIT_CMD_R               0x03
#define WIT_CMD_W               0x06

// Length
#define WIT_ME01_HEADER_LENGTH  2u
#define WIT_ME01_DATA_LENGTH    9u

extern float IMU_Yaw;

void Tripod_Receive(uint8_t* data);

#endif //DEVICE_WIT_JY_ME01_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
