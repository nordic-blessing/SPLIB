//
// Created by Icol_Lee on 2026/2/11.
//

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
