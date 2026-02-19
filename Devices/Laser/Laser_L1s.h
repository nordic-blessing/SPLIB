//
// Created by Icol_Lee on 2025/10/17.
//

#ifndef DEVICE_LASER_L1S_H
#define DEVICE_LASER_L1S_H

#include <stdint.h>

//协议定义
#define Laser_L1s_Tx_HEADER1       0xA5
#define Laser_L1s_Tx_HEADER2       0x5A
#define Laser_L1s_Tx_HEADER_LENGTH 2u
#define Laser_L1s_Tx_DATA_LENGTH   5u

#define Laser_L1s_Rx_HEADER1       0xB4
#define Laser_L1s_Rx_HEADER2       0x69
#define Laser_L1s_Rx_HEADER_LENGTH 2u
#define Laser_L1s_Rx_DATA_LENGTH   8u

extern uint32_t Laser_distance;

void Laser_L1s_Receive(uint8_t *data);

#endif //DEVICE_LASER_L1S_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
