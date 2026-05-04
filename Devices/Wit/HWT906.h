/**
 * @file    HWT906.h
 * @author  Icol_Lee (icolboom4@gmail.com)
 * @brief   
 * @version 0.1
 * @date    2026/5/3
 */

#ifndef DEVICE_WIT_HWT906_H
#define DEVICE_WIT_HWT906_H

#include <stdint.h>

#define WIT_HEADER        0x55
#define WIT_TYPE_EULER    0x53
#define WIT_HEADER_LENGTH 2u
#define WIT_DATA_LENGTH   11

extern float WIT_Yaw;

void WIT_Receive(uint8_t* data);

#endif //DEVICE_WIT_HWT906_H
