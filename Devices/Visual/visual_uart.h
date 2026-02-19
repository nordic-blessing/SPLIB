//
// Created by Icol_Lee on 2026/2/2.
//

#ifndef DEVICE_VISUAL_H
#define DEVICE_VISUAL_H

#include <stdint.h>
#include <string.h>

#define VISUAL_HEADER1        0xAA
#define VISUAL_HEADER2        0x55
#define VISUAL_HEADER_LENGTH    1u
#define VISUAL_TAIL1          0x0F
#define VISUAL_TAIL2          0x0A
#define VISUAL_TAIL_LENGTH      1u
#define VISUAL_DATA_LENGTH      28

typedef struct {
    float data1;
    float data2;
    float data3;
    float data4;
    float data5;
    float data6;
}Visual_t;

extern Visual_t visualData;

void Visual_Receive(uint8_t *data);

#endif //DEVICE_VISUAL_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
