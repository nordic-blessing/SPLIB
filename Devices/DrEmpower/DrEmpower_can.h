/**
  ******************************************************************************
  @file     DrEmpower_can.h
  @brief    大然一体化关节电机 CAN 驱动：
                - 功能
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-03-15 (Created) | 2026-3-1 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-19 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_DREMPOWER_CAN_H
#define DEVICE_DREMPOWER_CAN_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/

/* Exported function declarations ---------------------------------------------*/
void DrEmpower_Set_Angle(uint8_t id, float angle, uint16_t speed, uint16_t param, uint8_t mode);
void DrEmpower_Set_Speed(uint8_t id, float speed, uint16_t param, uint16_t mode);
void DrEmpower_Set_Torque(uint8_t id, float torque, uint16_t param, uint16_t mode);
void DrEmpower_Read_property(uint8_t id, uint16_t address, uint16_t data_type);
float DrEmpower_Get_Angle(uint8_t id);
float DrEmpower_Get_Speed(uint8_t id);
float DrEmpower_Get_Torque(uint8_t id);
float DrEmpower_Get_Temp(uint8_t id);
void DrEmpower_Receive(const uint8_t* rxdata);

#endif //DEVICE_DREMPOWER_CAN_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
