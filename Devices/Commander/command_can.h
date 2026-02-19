//
// Created by Icol_Lee on 2026/1/30.
//

#ifndef DEVICE_COMMAND_CAN_H
#define DEVICE_COMMAND_CAN_H

/* Includes ------------------------------------------------------------------*/
#if USE_SPLIB_FDCAN
    #include "fdcan.h"
#endif
#if USE_SPLIB_CAN
    #include "can.h"
#endif

/* Private macros ------------------------------------------------------------*/
#define COMMAND_CAN hfdcan2
#define COMMAND_ID  (0x214 + 1)

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
void command_transmit(uint8_t *pData);

#endif //DEVICE_COMMAND_CAN_H

 /************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
