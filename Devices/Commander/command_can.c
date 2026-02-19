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

void command_transmit(uint8_t *pData) {
    CAN_SendStdData(&COMMAND_CAN, 0x214 + COMMAND_ID, pData, FDCAN_DLC_BYTES_8);
}

void command_receive(const uint8_t *pData) {

}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
