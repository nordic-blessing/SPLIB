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
  @example
    MotorData_t A1_lift;

    Unitree_init(A1_lift, GO_M8010_6, 1);
    Unitree_get_motor(A1_lift);
    float lift_offset = A1_lift.Pos;

    for(;;){
        Unitree_set_angle(A1_lift, rad_des + lift_offset, 0.1f, 1.0f);
        DELAY(1);
    }
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
#include "splib.h"

#if USE_SPLIB_UNITREE

/* Includes ------------------------------------------------------------------*/
#include "Unitree_uart.h"

/* Private define ------------------------------------------------------------*/
#define UNITREE_GO_UART         huart2
#define UNITREE_A1_UART         huart2
#define UNITREE_GO_SEND()       HAL_UART_Transmit(&UNITREE_GO_UART, (uint8_t *) &motor->motorCmd_send.GO_M8010_6_motor_send_data, sizeof(motor->motorCmd_send.GO_M8010_6_motor_send_data), 1)
#define UNITREE_A1_SEND()       HAL_UART_Transmit(&UNITREE_UART, (uint8_t *) &motor->motorCmd_send.A1B1_motor_send_data, sizeof(motor->motorCmd_send.A1B1_motor_send_data), 1)
#define UNITREE_GO_RECEIVE()    HAL_UART_Receive(&UNITREE_GO_UART, (uint8_t *) &motor->GO_M8010_6_motor_recv_data, sizeof(motor->GO_M8010_6_motor_recv_data), 1)
#define UNITREE_A1_RECEIVE()    HAL_UART_Receive(&UNITREE_A1_UART, (uint8_t *) &motor->A1B1_motor_recv_data, sizeof(motor->A1B1_motor_recv_data), 1)
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 初始化电机
 * @param motor
 * @param motorType
 * @param id
 */
void Unitree_init(MotorData_t* motor, enum MotorType motorType, uint8_t id) {
    motor->motorType = motorType;

    motor->motor_id = id;
    motor->mode = 0;
    motor->Temp = 0;
    motor->MError = 0;
    motor->T = 0;
    motor->W = 0;
    motor->Pos = 0;
    motor->Acc = 0;
    motor->correct = 0;
    motor->calc_crc = 0;

    motor->motorCmd_send.motorType = motorType;
}

/**
 * 发送电机控制命令
 * @param motor
 */
void Unitree_send_cmd(MotorData_t* motor) {
    if (motor->motorType == GO_M8010_6) {
        UNITREE_GO_SEND();
    } else if (motor->motorType == A1) {
        UNITREE_A1_SEND();
    }
}

/**
 * 回读电机反馈信息
 * @param motor
 */
void Unitree_receive_data(MotorData_t* motor) {
    // 接收数据
    if (motor->motorType == GO_M8010_6) {
        UNITREE_GO_RECEIVE();
    } else if (motor->motorType == A1) {
        Unitree_A1_RECEIVE();
    }

    // 解析数据
    if (Unitree_extract_data(motor) == HAL_ERROR) {
        // Error_Handler();
    }
}

/**
 * 锁死模式，用于开机获取电机位置
 * @param motor
 */
void Unitree_get_motor(MotorData_t* motor) {
    motor->motorCmd_send.id = motor->motor_id;
    motor->motorCmd_send.mode = 0;
    motor->motorCmd_send.K_P = 0;
    motor->motorCmd_send.K_W = 0;
    motor->motorCmd_send.Pos = 0;
    motor->motorCmd_send.W = 0;
    motor->motorCmd_send.T = 0;
    Unitree_modify_data(&motor->motorCmd_send);
    Unitree_send_cmd(motor);
    Unitree_receive_data(motor);
}

/**
 * 位置模式
 * @param motor
 * @param rad   输出轴角度(弧度制)
 * @param kp    刚度系数
 * @param kw    阻尼系数
 */
void Unitree_set_angle(MotorData_t* motor, float rad, float kp, float kw){
    motor->motorCmd_send.id = motor->motor_id;
    if(motor->motorType == GO_M8010_6){
        motor->motorCmd_send.mode == 1;
        motor->motorCmd_send.Pos = rad * 6.33f;
    }else if(motor->motorType == A1) {
        motor->motorCmd_send.mode = 10;
        motor->motorCmd_send.Pos = rad * 9.0f;
    }
    motor->motorCmd_send.K_P = kp;
    motor->motorCmd_send.K_W = kw;
    motor->motorCmd_send.W = 0;
    motor->motorCmd_send.T = 0;
    Unitree_modify_data(&motor->motorCmd_send);
    Unitree_send_cmd(motor);
    Unitree_receive_data(motor);
}

/**
 * 速度模式
 * @param motor
 * @param w     输出轴速度(弧度制)
 * @param kw    阻尼系数
 */
void Unitree_set_speed(MotorData_t* motor, float w, float kw){
    motor->motorCmd_send.id = motor->motor_id;
    if(motor->motorType == GO_M8010_6){
        motor->motorCmd_send.mode == 1;
        motor->motorCmd_send.W = w * 6.33f;
    }else if(motor->motorType == A1) {
        motor->motorCmd_send.mode = 10;
        motor->motorCmd_send.W = w * 9.0f;
    }
    motor->motorCmd_send.K_P = 0;
    motor->motorCmd_send.K_W = kw;
    motor->motorCmd_send.Pos = 0;
    motor->motorCmd_send.T = 0;
    Unitree_modify_data(&motor->motorCmd_send);
    Unitree_send_cmd(motor);
    Unitree_receive_data(motor);
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
