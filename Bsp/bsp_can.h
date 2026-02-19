//
// Created by Icol_Lee on 2025/9/27.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#if USE_SPLIB_FDCAN
#include "fdcan.h"
#endif
#if USE_SPLIB_CAN
#include "can.h"
#endif

/* Private macros ------------------------------------------------------------*/
#define CanFilter_0     (0  << 3)
#define CanFilter_1     (1  << 3)
#define CanFilter_2     (2  << 3)
#define CanFilter_3     (3  << 3)
#define CanFilter_4     (4  << 3)
#define CanFilter_5     (5  << 3)
#define CanFilter_6     (6  << 3)
#define CanFilter_7     (7  << 3)
#define CanFilter_8     (8  << 3)
#define CanFilter_9     (9  << 3)
#define CanFilter_10    (10 << 3)
#define CanFilter_11    (11 << 3)
#define CanFilter_12    (12 << 3)
#define CanFilter_13    (13 << 3)
#define CanFilter_14    (14 << 3)
#define CanFilter_15    (15 << 3)
#define CanFilter_16    (16 << 3)
#define CanFilter_17    (17 << 3)
#define CanFilter_18    (18 << 3)
#define CanFilter_19    (19 << 3)
#define CanFilter_20    (20 << 3)
#define CanFilter_21    (21 << 3)
#define CanFilter_22    (22 << 3)
#define CanFilter_23    (23 << 3)
#define CanFilter_24    (24 << 3)
#define CanFilter_25    (25 << 3)
#define CanFilter_26    (26 << 3)
#define CanFilter_27    (27 << 3)

#define CanFifo_0       (0 << 2)
#define CanFifo_1       (1 << 2)

#define Can_StdId       (0 << 1)
#define Can_ExtId       (1 << 1)

#define Can_DataType    (0 << 0)
#define Can_RemoteType  (1 << 0)

#if USE_SPLIB_FDCAN
#define CAN_HandleTypeDef   FDCAN_HandleTypeDef
#define CAN_RxHeaderTypeDef FDCAN_RxHeaderTypeDef
#endif

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
}CAN_RxBuffer;

/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
void CAN_Start_IT(CAN_HandleTypeDef* hcan, uint8_t FIFOx, void (*pFunc)(CAN_RxBuffer*));
void CAN_Filter_Mask_Config(CAN_HandleTypeDef* hcan, uint8_t para, uint32_t Id, uint32_t Mask);
void CAN_SendStdData(CAN_HandleTypeDef* hcan, uint16_t StdId, uint8_t* pData, uint8_t Len);
void CAN_SendExtData(CAN_HandleTypeDef* hcan, uint16_t ExtId, uint8_t* pData, uint8_t Len);

#endif //BSP_CAN_H
