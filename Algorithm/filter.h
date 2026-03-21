/**
  ******************************************************************************
  @file     filter.h
  @brief    滤波算法：
                -
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2026-03-14 (Created) | 2026-03-14 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-03-14 [v1.0] Icol Boom: 创建初始版本
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef ALGORITHM_FILTER_H
#define ALGORITHM_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/* Private macros ------------------------------------------------------------*/
#define NUM_SAMPLES 8
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
// 低通滤波
typedef struct {
    float ts;       //采样周期(s)
    float fc;       //截至频率(hz)
    float lastYn;   //上一次滤波值
    float alpha;    //滤波系数
} Filter_LP_t;

typedef struct {
    uint16_t samples[NUM_SAMPLES]; // 存储最近的RPM样本
    uint32_t sum;                   // 当前窗口内RPM值的总和
    uint8_t index;                  // 当前写入的位置
    uint8_t count;                  // 当前样本数量
} Filter_MA_t;

/* Exported variables ---------------------------------------------------------*/
/* Exported function declarations ---------------------------------------------*/
void Filter_Init_LP(Filter_LP_t* filter, float ts, float fc);
float LP_Filter(Filter_LP_t* filter, float data);
void Filter_Init_MAF(Filter_MA_t *maf);
float MAF_Filter(Filter_MA_t *maf, uint16_t new_data);


#endif //ALGORITHM_FILTER_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
