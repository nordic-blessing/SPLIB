/**
  ******************************************************************************
  @file     pid.c
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
  ------------------------------------------------------------------------------
  @example 
    - PID初始化 : 调用`initPID()`, 配置PID控制器参数及限幅
        `initPID(&pid, 100.0f, 500.0f, 0.5f)` // 最大输出100，最大积分500，死区0.5
    - 设置PID参数 : 调用`setPIDParam()`, 配置kp/ki/kd
        `setPIDParam(&pid, 8.0f, 0.1f, 0.5f)` // kp=8, ki=0.1, kd=0.5
    - 设置目标值 : 调用`setPIDTarget()`, 设置PID控制目标
        `setPIDTarget(&pid, 500.0f)` // 设定目标值为500
    - 增量式PID计算 : 调用`updatePID_Incremental()`, 实时更新PID输出
        `updatePID_Incremental(&pid, motor_speed)` // 输入当前电机转速，计算增量式PID输出
    - 位置式PID计算 : 调用`updatePID_Position()`, 实时更新PID输出
        `updatePID_Position(&pid, motor_angle)` // 输入当前电机角度，计算位置式PID输出
  ------------------------------------------------------------------------------
  @attention
    - 请根据实际项目需求，在`pid.h`中添加`PID_t`实例化
    - 增量式PID依赖上一次输出值(output_last)和历史误差(pre/last)，首次使用需确保初始值为0
    - 位置式PID包含积分项，需通过MAX_ERROR_INTEGRAL限制积分饱和，避免输出失控
    - 死区参数(deadZone)非零时，误差绝对值小于该值则置0，可减少小误差下的频繁调节
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#include "splib.h"

#if USE_SPLIB_PID

/* Includes ------------------------------------------------------------------*/
#include "pid.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
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
 * @param ptr
 * @param kp
 * @param ki
 * @param kd
 */
void setPIDParam(PID_t* ptr, float kp, float ki, float kd) {
    ptr->kp = kp;
    ptr->ki = ki;
    ptr->kd = kd;
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
 * @param ptr
 * @param input
 */
void updatePID_Incremental(PID_t* ptr, float input){
    ptr->input = input;
    ptr->error.now = ptr->target - ptr->input;

    if (fabsf(ptr->error.now) < ptr->deadZone) {
        ptr->error.now = 0;
    }

    ptr->output = ptr->output_last
                  + ptr->kp * (ptr->error.now - ptr->error.last)
                  + ptr->ki * ptr->error.now
                  + ptr->kd * (ptr->error.now - 2 * ptr->error.last + ptr->error.pre);
    ptr->output = limitOutput(ptr->output, ptr->MAX_OUTPUT);

    ptr->error.pre = ptr->error.last;
    ptr->error.last = ptr->error.now;
    ptr->output_last = ptr->output;
}

/**
 * 位置式pid
 * @param ptr
 * @param input
 */
void updatePID_Position(PID_t* ptr, float input) {
    ptr->input = input;
    ptr->error.now = ptr->target - ptr->input;

    if (fabsf(ptr->error.now) < ptr->deadZone) {
        ptr->error.now = 0;
    }

    ptr->error.integral += ptr->error.now;
    ptr->error.integral = limitOutput(ptr->error.integral, ptr->MAX_ERROR_INTEGRAL);

    ptr->output = ptr->kp * ptr->error.now
                  + ptr->ki * ptr->error.integral
                  + ptr->kd * (ptr->error.now - ptr->error.last);
    ptr->output = limitOutput(ptr->output, ptr->MAX_OUTPUT);

    ptr->error.last = ptr->error.now;
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
