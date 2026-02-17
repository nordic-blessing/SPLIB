//
// Created by 30958 on 2024/8/21.
//
#include "splib.h"

#if USE_SPLIB_PID

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include <math.h>

/**
 * 限幅
 * @param output
 * @param MAX_OUTPUT_ABS 限幅范围
 * @return
 */
float limitOutput(float output, float MAX_OUTPUT_ABS) {
    if (output > (+MAX_OUTPUT_ABS)) return (+MAX_OUTPUT_ABS);
    if (output < (-MAX_OUTPUT_ABS)) return (-MAX_OUTPUT_ABS);
    return output;
}

/**
 * 初始化pid
 * @param pid
 * @param MAX_OUTPUT    最大输出值
 * @param MAX_E_I       最大误差积分值
 */
void initPID(PID_t* pid, float MAX_OUTPUT, float MAX_E_I, float deadZone) {
    // param
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;

    // TIO
    pid->target = 0;
    pid->input = 0;
    pid->output = 0;

    // error
    pid->error.now = 0;
    pid->error.last = 0;
    pid->error.integral = 0;

    //limit
    pid->MAX_OUTPUT = MAX_OUTPUT;
    pid->MAX_ERROR_INTEGRAL = MAX_E_I;
    pid->deadZone = deadZone;
}

/**
 * 设置pid参数
 * @param pid
 * @param kp
 * @param ki
 * @param kd
 */
void setPIDParam(PID_t* pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * 设置pid目标值
 * @param pid
 * @param target
 */
void setPIDTarget(PID_t* pid, float target){
    pid->target = target;
}

/**
 * 增量式pid
 * @param pid
 * @param input
 */
void updatePID_Incremental(PID_t* pid, float input){
    pid->input = input;
    pid->error.now = pid->target - pid->input;

    if (fabsf(pid->error.now) < pid->deadZone) {
        pid->error.now = 0;
    }

    pid->output = pid->output_last
                  + pid->kp * (pid->error.now - pid->error.last)
                  + pid->ki * pid->error.now
                  + pid->kd * (pid->error.now - 2 * pid->error.last + pid->error.pre);
    pid->output = limitOutput(pid->output,pid->MAX_OUTPUT);

    pid->error.pre = pid->error.last;
    pid->error.last = pid->error.now;
    pid->output_last = pid->output;
}

/**
 * 位置式pid
 * @param pid
 * @param input
 */
void updatePID_Position(PID_t* pid, float input) {
    pid->input = input;
    pid->error.now = pid->target - pid->input;

    if (fabsf(pid->error.now) < pid->deadZone) {
        pid->error.now = 0;
    }

    pid->error.integral += pid->error.now;
    pid->error.integral = limitOutput(pid->error.integral, pid->MAX_ERROR_INTEGRAL);

    pid->output = pid->kp * pid->error.now
                  + pid->ki * pid->error.integral
                  + pid->kd * (pid->error.now - pid->error.last);
    pid->output = limitOutput(pid->output, pid->MAX_OUTPUT);

    pid->error.last = pid->error.now;
}
#endif
