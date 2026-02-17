//
// Created by Icol_Lee on 2025/9/20.
//

#ifndef SPLIB_CONFIG_H
#define SPLIB_CONFIG_H

/* Algorithm ----------------------------------------------*/
#define USE_SPLIB_PID               0
#define USE_SPLIB_TRAJPLANNER       0

/* Bsp ----------------------------------------------------*/
#define USE_SPLIB_CAN               0
#define USE_SPLIB_FDCAN             0
#define USE_SPLIB_LED               0
#define USE_SPLIB_UART              0

/* Devices  -----------------------------------------------*/
#define USE_SPLIB_CONMMAND          0
#define USE_SPLIB_DJI               0
#define USE_SPLIB_EREMPOWER         0
#define USE_SPLIB_LASER_L1S         0
#define USE_SPLIB_REMOTER_SBUS      0
#define USE_SPLIB_ROBOSTRIDE        0
#define USE_SPLIB_TJC_UART          0
#define USE_SPLIB_UNITREE           0
#define USE_SPLIB_VESC              0
#define USE_SPLIB_VISUAL_UART       0
#define USE_SPLIB_VOFA_DEBUG        0
#define USE_SPLIB_VOFA_PRINTF       0
#define USE_SPLIB_WIT_JY_ME01       0


/* Algorithm header begin */
#if USE_SPLIB_PID
    #include "Algorithm/pid.h"
#endif
#if USE_SPLIB_TRAJPLANNER
    #include "Algorithm/TrajPlanner.h"
#endif

/* Bsp header begin */
#if USE_SPLIB_FDCAN | USE_SPLIB_CAN
    #include "Bsp/bsp_can.h"
#endif
#if USE_SPLIB_LED
    #include "BSP/bsp_led.h"
#endif
#if USE_SPLIB_UART
    #include "BSP/bsp_uart.h"
#endif
/* Bsp header end */

/* Devices header begin */
#if USE_SPLIB_CONMMAND
    #include "Devices/Commander/command_can.h"
#endif
#if USE_SPLIB_DJI
    #include "Devices/Dji/M3508.h"
#endif
#if USE_SPLIB_EREMPOWER
    #include "Devices/DrEmpower/DrEmpower_can.h"
#endif
#if USE_SPLIB_LASER_L1S
    #include "Devices/Laser/Laser_L1s.h"
#endif
#if USE_SPLIB_REMOTER_SBUS
    #include "Devices/Remoter/sbus.h"
#endif
#if USE_SPLIB_ROBOSTRIDE
    #include "Devices/Robstride/Robstride.h"
#endif
#if USE_SPLIB_TJC_UART
    #include "Devices/TJC/tjc_uart_hmi.h"
#endif
#if USE_SPLIB_UNITREE
    #include "Devices/Unitree/MotorOutput.h"
#endif
#if USE_SPLIB_VESC
    #include "Devices/VESC/VESC.h"
#endif
#if USE_SPLIB_VISUAL_UART
    #include "Devices/Visual/visual_uart.h"
#endif
#if USE_SPLIB_VOFA_DEBUG
    #include "Devices/VOFA/uart_debug.h"
#endif
#if USE_SPLIB_VOFA_PRINTF
    #include "Devices/VOFA/uart_printf.h"
#endif
#if USE_SPLIB_WIT_JY_ME01
    #include "Devices/Wit/JY-ME01.h"
#endif
/* Devices header end */

#endif //SPLIB_CONFIG_H
