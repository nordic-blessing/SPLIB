/**
  ******************************************************************************
  @file     pid.h
  @brief    pid控制算法： 
                - 输出限幅处理
                - PID控制器初始化（配置参数/限幅/死区）
                - PID参数（kp/ki/kd）及目标值设置
                - 增量式PID计算
                - 位置式PID计算（含积分限幅）
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2024-08-21 (Created) | 2026-02-19 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-19 [v1.0] Icol Boom: 创建初始版本
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#ifndef ALGORITHM_PID_H
#define ALGORITHM_PID_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
//误差
typedef struct {
    float now;          //当前误差
    float last;         //上一次误差
    float pre;          //
    float integral;     //累积误差
}Error;

typedef struct {
    float kp, ki, kd;                           // kp ki kd
    float target, input, output, output_last;   // T I O
    Error error;                                // 误差

    float MAX_OUTPUT;           // 最大输出
    float MAX_ERROR_INTEGRAL;   // 最大积分
    float deadZone;             // 死区
}PID_t;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
// 实例化
extern PID_t pid;

/* Exported function declarations ---------------------------------------------*/
void initPID(PID_t* pid, float MAX_OUTPUT, float MAX_E_I, float deadZone);
void setPIDParam(PID_t* pid, float kp, float ki, float kd);
void setPIDTarget(PID_t* pid, float target);
void updatePID_Position(PID_t* pid, float input);
void updatePID_Speed(PID_t* pid, float input);

#endif //ALGORITHM_PID_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
