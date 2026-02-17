#include "TrajPlanner.h"

#define TRAJ_SIGMOID    1

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
