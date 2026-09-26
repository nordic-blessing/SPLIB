/**
******************************************************************************
  @file     Unitree_uart.c
  @brief    宇树电机驱动
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2024-11-22 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2024-11-22 [v1.0] Icol Boom: 创建初始版本
  ------------------------------------------------------------------------------
  @attention
    - 宇树电机的核心是MIT控制，τ_ref = Kp*(θ_des - θ_act) + Kd*(dθ_des - dθ_act) + τ_ff
    - 驱动依赖于串口外设，需要正确配置串口
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
        让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_UNITREE_MOTOROUPUT_H
#define DEVICE_UNITREE_MOTOROUPUT_H

#include "unitreeMotor.h"
#include "usart.h"

void Unitree_init(MotorData_t* motor, enum MotorType motorType, uint8_t id);
void Unitree_receive_data(MotorData_t* motor);
void Unitree_get_motor(MotorData_t* motor);
void Unitree_set_angle(MotorData_t* motor, float rad, float kp, float kw);
void Unitree_set_speed(MotorData_t* motor, float w, float kw);

#endif //DEVICE_UNITREE_MOTOROUPUT_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
