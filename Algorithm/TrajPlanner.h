/**
  ******************************************************************************
  @file     TrajPlanner.h
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
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef ALGORITHM_TRAJPLANNER_H
#define ALGORITHM_TRAJPLANNER_H

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

typedef struct {
    float current;  // 当前
    float start;    // 初始
    float target;   // 目标
    float rate;     // 最大速度
    int8_t dir;     // 运动方向

    float Tick;     // 时间戳
    float maxTimes; // 控制总时间
    float delta_t;

    float A,B,C,D;  // 曲线参数
    bool arrived;   // 是否到达目标状态（作用重规划）

    float rate_max; // 最大速度限制
}Traj_t;

void Traj_Init(Traj_t *traj, float start, float rate_max, float delta);
void Traj_Update(Traj_t* traj);
void Traj_SetTarget(Traj_t* traj, float target);

#endif //ALGORITHM_TRAJPLANNER_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
