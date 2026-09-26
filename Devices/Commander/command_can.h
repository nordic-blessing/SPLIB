/**
******************************************************************************
  @file     command_can.h
  @brief    私有CAN协议
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-01-30 (Created) | 2026-04-28 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-01-30 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ------------------------------------------------------------------------------
  @example

  ------------------------------------------------------------------------------
  @attention
    - 驱动依赖于`bsp_can.c/h`，请务必在`splib_config.h`中使能`USE_SPLIB_CAN`或
        `USE_SPLIB_FDCAN`
    - 代码未经测试，谨慎使用
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
    让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_COMMAND_CAN_H
#define DEVICE_COMMAND_CAN_H

/* Includes -----------------------------------------------------------------*/
#if USE_SPLIB_FDCAN
    #include "fdcan.h"
#endif
#if USE_SPLIB_CAN
    #include "can.h"
#endif

/* Private macros -----------------------------------------------------------*/
#define COMMAND_CAN         hfdcan1

#define COMMAND_MASTER      0   // 1:Master 0:Slave

#define DEVICE_ADDR         0b010    // 用户自定义 3 bit 设备地址

#define DEVICE_ID           ((COMMAND_MASTER << 3)|DEVICE_ADDR) // 设备区别ID

/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
// 消息ID
#define COMMAND_CMD_0       0x00
#define COMMAND_CMD_1       0x01
#define COMMAND_CMD_2       0x02
#define COMMAND_CMD_3       0x03
#define COMMAND_CMD_4       0x04
#define COMMAND_CMD_5       0x05
#define COMMAND_CMD_6       0x06
#define COMMAND_CMD_7       0x07
#define COMMAND_CMD_8       0x08
#define COMMAND_CMD_9       0x09
#define COMMAND_CMD_10      0x0A
#define COMMAND_CMD_11      0x0B
#define COMMAND_CMD_12      0x0C
#define COMMAND_CMD_13      0x0D
#define COMMAND_CMD_14      0x0E
#define COMMAND_CMD_15      0x0F
#define COMMAND_CMD_16      0x10
#define COMMAND_CMD_17      0x11
#define COMMAND_CMD_18      0x12
#define COMMAND_CMD_19      0x13
#define COMMAND_CMD_20      0x14
#define COMMAND_CMD_21      0x15
#define COMMAND_CMD_22      0x16
#define COMMAND_CMD_23      0x17
#define COMMAND_CMD_24      0x18
#define COMMAND_CMD_25      0x19
#define COMMAND_CMD_26      0x1A
#define COMMAND_CMD_27      0x1B
#define COMMAND_CMD_28      0x1C
#define COMMAND_CMD_29      0x1D
#define COMMAND_CMD_30      0x1E
#define COMMAND_CMD_31      0x1F

/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/
void command_transmit(uint8_t CMD_ID, uint8_t Data_type, uint8_t *pData);
void command_receive(CAN_RxBuffer* rxBuffer);

#endif //DEVICE_COMMAND_CAN_H

 /************************** COPYRIGHT(C) Pangolin Robot Lab **************************/
