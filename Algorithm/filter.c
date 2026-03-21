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
  ------------------------------------------------------------------------------
  @example
    -
  ------------------------------------------------------------------------------
  @attention
    -
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#include "splib.h"

#if USE_SPLIB_FILTER
/* Includes ------------------------------------------------------------------*/
#include "filter.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

// RC 低通滤波
/**
 * 初始化
 * @param filter    过滤器句柄
 * @param ts        采样时间（s）
 * @param fc        截止频率（hz）
 */
void Filter_Init_LPF(Filter_LP_t* filter, float ts, float fc) {
    float b = 2.0f * (float)M_PI * fc * ts;

    filter->ts = ts;
    filter->fc = fc;
    filter->lastYn = 0;
    filter->alpha = b / (b + 1);
}

/**
 * 输出过滤后结果
 * @param filter    过滤器句柄
 * @param data      采样数据
 * @return
 */
float LPF_Filter(Filter_LP_t* filter, float data) {
    float tem = filter->lastYn + (filter->alpha * (data - filter->lastYn));
    filter->lastYn = tem;
    return tem;
}


// 滑动窗口滤波
/**
 *
 * @param maf
 */
void Filter_Init_MAF(Filter_MA_t *maf)
{
    for (int i = 0; i < NUM_SAMPLES; i++) {
        maf->samples[i] = 0;
    }
    maf->sum = 0;
    maf->index = 0;
    maf->count = 0;
}

/**
 *
 * @param maf
 * @param new_data
 * @return
 */
float MAF_Filter(Filter_MA_t *maf, uint16_t new_data)
{
    // 从总和中减去即将被替换的旧数据
    maf->sum -= maf->samples[maf->index];

    // 替换为新数据
    maf->samples[maf->index] = new_data;

    // 加上新数据到总和
    maf->sum += new_data;

    // 更新样本计数，直到达到窗口大小
    if (maf->count < NUM_SAMPLES) {
        maf->count++;
    }

    // 更新索引，并在达到窗口大小时回绕
    maf->index = (maf->index + 1) % NUM_SAMPLES;

    // 计算平均值
    return maf->sum / maf->count;
}


#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
