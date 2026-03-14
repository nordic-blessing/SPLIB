#pragma once

#include "splib_config.h"

#define USE_FREERTOS    1

#if USE_FREERTOS
    #include "cmsis_os.h"
    #define DELAY(x)        osDelay(x)
    #define MALLOC(size)    pvPortMalloc(size)
    #define FREE(ptr)       vPortFree(ptr)
#else
    #define DELAY(x)        HAL_Delay(x)
    #define MALLOC(size)    malloc(size)
    #define FREE(ptr)       free(ptr)
#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
