/**
  ******************************************************************************
  @file     mA_Acquisition.c
  @brief    4-20mA电流采集转RS485数字输出的Modbus协议驱动：
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2026-03-04 (Created) | 2026-03-04 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-03-04 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef SPLIB_DEVICE_MA_ACQUISITION_H
#define SPLIB_DEVICE_MA_ACQUISITION_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define ACQUISITION_RECEIVE_PL      7u

#define ACQUISITION_FUNC            0x03
#define ACQUISITION_DATA_LENGTH     0x02
#define ACQUISITION_HEADER_LENGTH   2u

#define ACQUISITION_ADDR_VALUE      0x00
#define ACQUISITION_ADDR_TEMP       0x01
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t id;
    uint16_t value;
} Acquisition_t;
/* Exported variables ---------------------------------------------------------*/
extern Acquisition_t DT35_FM;

/* Exported function declarations ---------------------------------------------*/
void Acquisition_GetValue(Acquisition_t* pAcquisition);

#endif //SPLIB_DEVICE_MA_ACQUISITION_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
