/**
  ******************************************************************************
  @file     TrajPlanner.c
  @brief    曲线轨迹规划算法： 
                - 轨迹规划器初始化（配置起始值/最大速率/时间步长）
                - 三段式Sigmoid曲线计算（加速/匀速/减速阶段）
                - 轨迹规划实时更新（参数计算/曲线迭代）
                - 规划目标值设置（支持中途修改目标）
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2024-08-22 (Created) | 2026-02-19 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-19 [v1.0] Icol Boom: 创建初始版本，完成三段式Sigmoid曲线轨迹规划基础功能开发
  ------------------------------------------------------------------------------
  @example 
    - 轨迹规划器初始化 : 调用`Traj_Init()`, 配置轨迹规划基础参数
        `Traj_Init(&pos_traj, 0.0f, 5.0f, 0.001f)` // 起始值0，最大速率5，时间步长0.001s
    - 设置规划目标值 : 调用`Traj_SetTarget()`, 设定轨迹目标位置
        `Traj_SetTarget(&pos_traj, 100.0f)` // 规划目标值为100.0
    - 实时更新轨迹 : 调用`Traj_Update()`, 迭代计算当前轨迹值
        `Traj_Update(&pos_traj)` // 在循环函数中调用，更新current为当前规划值
    - 获取当前规划值 : 直接读取traj结构体的current字段
        `float current_pos = pos_traj.current;` // 获取当前规划的位置值
  ------------------------------------------------------------------------------
  @attention
    - 时间步长delta_t建议与函数循环周期一致
    - 最大速率rate_max需根据实际场景配置，过大会导致Sigmoid曲线加速阶段斜率超限
    - 目标值与起始值差值过小时（小于1e-6），轨迹规划会直接判定为到达目标，无曲线过渡
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#include "splib.h"

#if USE_SPLIB_TRAJPLANNER

/* Includes ------------------------------------------------------------------*/
#include "TrajPlanner.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * 曲线规划初始化
 * @param traj
 * @param start
 * @param rate
 */
void Traj_Init(Traj_t *traj, float start, float rate_max, float delta) {
    traj->maxTimes = 0.0f;
    traj->Tick = 0.0f;
    traj->delta_t = delta;

    traj->current = 0.0f;
    traj->start = start;
    traj->target = 0.0f;
    traj->rate = 0.0f;

    traj->A = 0.0f;
    traj->B = 0.0f;
    traj->C = 0.0f;
    traj->D = 0.0f;

    traj->arrived = true;
    traj->rate_max = rate_max;
}

/**
 * 三段式sigmoid曲线规划
 * @param traj
 */
void Traj_Calc_Sigmoid(Traj_t* traj) {
    if (traj->Tick < traj->C) { // 加速阶段
        traj->current = traj->D
                        + traj->A / (1.f + expf((-traj->B) * (traj->Tick - traj->C)));
    } else if (traj->Tick < (traj->maxTimes - traj->C)) { // 匀速阶段
        traj->current = traj->A / 2.f + traj->D
                        + traj->dir * traj->rate * (traj->Tick - traj->C);
    } else { // 减速阶段
        traj->current = traj->target - traj->A
                        + traj->A / (1.f + expf((-traj->B) * (traj->Tick - traj->maxTimes + traj->C)));
    }
    traj->Tick += traj->delta_t;
}

/**
 * 曲线规划更新
 * @param traj
 */
void Traj_Update(Traj_t* traj) {
    //初始化阶段，计算参数
    if (traj->arrived == true) {
        float delta = traj->target - traj->start;
        traj->dir = delta > 0 ? 1 : -1;
        if (fabsf(delta) > 1e-6f) {
            //初始化位置曲线
            traj->A = (delta) / 2.f;                                        // sigmoid 幅值
            float rate = fabsf(traj->A) / 0.5f;
            traj->rate = rate > traj->rate_max ? traj->rate_max : rate;     // 计算最大速度
            traj->B = fabsf(4.f * traj->rate / traj->A);                    // (A*B)/4 曲线的最大斜率
            float eps_pos = fabsf(traj->A / 400.f);
            traj->C = logf((fabsf(traj->A) - eps_pos) / eps_pos) / traj->B; // 起始 t 偏移值
            traj->D = traj->start;                                          // 起始 y 偏移值
            traj->maxTimes = fabsf(traj->A / traj->rate + 2.f * traj->C);   // 计算规划时间
            traj->Tick = 0.f;                                               // 初始化时间戳
            traj->arrived = false;
        }
    } else {
        if (traj->Tick < traj->maxTimes) { // 规划阶段
#if TRAJ_SIGMOID
            Traj_Calc_Sigmoid(traj);
#endif
        } else { // 结束规划
            traj->current = traj->target;
            traj->start = traj->target;
            traj->arrived = true;
            traj->Tick = 0.f;
            traj->maxTimes = 0.f;
        }
    }
}

/**
 * 设置规划目标
 * @param traj
 * @param target
 */
void Traj_SetTarget(Traj_t* traj, float target) {
    traj->target = target;
    if (!traj->arrived) {
        traj->start = traj->current;
    }
    traj->arrived = true;
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
