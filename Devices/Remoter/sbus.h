//
// Created by 30958 on 2026/1/14.
//

#ifndef DEVICE_TEST_SBUS_H
#define DEVICE_TEST_SBUS_H


#include <stdint.h>
#include <stddef.h>

//协议定义
#define REMOTER_HEADER        0x0F
#define REMOTER_HEADER_LENGTH 1u
#define REMOTER_TAIL          0x00
#define REMOTER_TAIL_LENGTH   1u
#define REMOTER_DATA_LENGTH   25

typedef struct {
    float CH1;
    float CH2;
    float CH3;
    float CH4;
    float CH5;
    float CH6;
}Remoter_sbus;

extern Remoter_sbus RemoterData;

void Remoter_Receive(uint8_t* data);

#endif //DEVICE_TEST_SBUS_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
