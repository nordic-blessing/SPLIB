//
// Created by Icol_Lee on 2024/8/21.
//
#ifndef ALGORITHM_PID_H
#define ALGORITHM_PID_H

#include "main.h"

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

// 实例化
extern PID_t posSwitch;

void initPID(PID_t* pid, float MAX_OUTPUT, float MAX_E_I, float deadZone);
void setPIDParam(PID_t* pid, float kp, float ki, float kd);
void setPIDTarget(PID_t* pid, float target);
void updatePID_Position(PID_t* pid, float input);
void updatePID_Speed(PID_t* pid, float input);

#endif //ALGORITHM_PID_H
