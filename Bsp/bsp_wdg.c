/**
  ******************************************************************************
  @file     bsp_wdg.c
  @brief    STM32(HAL) wdg驱动：
                - 窗口看门狗早唤醒处理
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2026-03-13 (Created) | 2026-03-13 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-03-13 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ------------------------------------------------------------------------------
  @example
    - 在此处重定义早唤醒中断函数即可
  ------------------------------------------------------------------------------
  @attention
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
    - 本驱动仅测试了STM32F4/G4系列的部分型号，依赖STM32 HAL库底层初始化
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#include "splib.h"

#if USE_SPLIB_WDG

#include "bsp_wdg.h"

uint8_t wwdg_reset[8]={0};

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg){
    /* 添加 WWDG 早唤醒处理 */
    // 示例：uart_printf("r\r\n");
    command_transmit(COMMAND_CMD_RESET, 0, wwdg_reset); // 通过 CAN 通知所有设备
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
