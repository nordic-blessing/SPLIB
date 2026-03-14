//
// Created by Icol_Lee on 2026/1/30.
//

#include "splib.h"

#if USE_SPLIB_CONMMAND

/* Includes ------------------------------------------------------------------*/
#include "command_can.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

void command_transmit(uint8_t CMD_ID, uint8_t Data_type, uint8_t *pData) {
    uint16_t std_id = DEVICE_ID | (CMD_ID << 4) | (Data_type << 2);
    CAN_SendStdData(&COMMAND_CAN, std_id, pData, 8);
}

void command_receive(const uint8_t *pData) {
    // 处理主控制器信息
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
