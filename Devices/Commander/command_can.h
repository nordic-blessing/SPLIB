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
#define COMMAND_CAN         hfdcan2

#define COMMAND_MASTER      1
#define COMMAND_SLAVE       0

#define DEVICE_ID           (COMMAND_SLAVE<<10)|(0x0<<8)

#define COMMAND_CMD_RESET   0xFF


/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
void command_transmit(uint8_t CMD_ID, uint8_t Data_type, uint8_t *pData);
void command_receive(const uint8_t *pData);

#endif //DEVICE_COMMAND_CAN_H

 /************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
