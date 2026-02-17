//
// Created by Icol_Lee on 2025/6/22.
//
#include "splib.h"

#if USE_SPLIB_UNITREE

#include "MotorOutput.h"

MotorCmd_t cmd[MOTOR_ID_NUM] = {0};

void unitree_init(MotorData_t* motor, enum MotorType motorType, uint8_t id) {
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
 * @param motor_id
 */
void unitree_send_cmd(MotorData_t* motor) {
    if (motor->motorType == GO_M8010_6) {
        HAL_UART_Transmit(&UNITREE_UART, (uint8_t *) &motor->motorCmd_send.GO_M8010_6_motor_send_data,
                          sizeof(motor->motorCmd_send.GO_M8010_6_motor_send_data), 1);
    } else if (motor->motorType == A1) {
        HAL_UART_Transmit(&UNITREE_UART, (uint8_t *) &motor->motorCmd_send.A1B1_motor_send_data,
                          sizeof(motor->motorCmd_send.A1B1_motor_send_data), 1);
    }
}

/**
 * 回读电机反馈信息
 * @param id
 */
void unitree_receive_data(MotorData_t* motor) {
    if (motor->motorType == GO_M8010_6) {
        HAL_UART_Receive(&UNITREE_UART, (uint8_t *) &motor->GO_M8010_6_motor_recv_data,
                         sizeof(motor->GO_M8010_6_motor_recv_data), 1);
    } else if (motor->motorType == A1) {
        HAL_UART_Receive(&UNITREE_UART, (uint8_t *) &motor->A1B1_motor_recv_data,
                         sizeof(motor->A1B1_motor_recv_data), 1);
    }
    if (unitree_extract_data(motor) == HAL_ERROR) {
        // Error_Handler();
    }
}

/**
 * 锁死模式，用于开机获取电机位置
 * @param motor_id
 */
void unitree_get_motor(MotorData_t* motor) {
    motor->motorCmd_send.id = motor->motor_id;
    motor->motorCmd_send.mode = 0;
    motor->motorCmd_send.K_P = 0;
    motor->motorCmd_send.K_W = 0;
    motor->motorCmd_send.Pos = 0;
    motor->motorCmd_send.W = 0;
    motor->motorCmd_send.T = 0;
    unitree_modify_data(&motor->motorCmd_send);
    unitree_send_cmd(motor);
    unitree_receive_data(motor);
}

/**
 * 位置模式
 * @param motor_id
 * @param rad   输出轴角度(弧度制)
 * @param kp    刚度系数
 * @param kw    阻尼系数
 */
void unitree_set_angle(MotorData_t* motor, float rad, float kp, float kw){
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
    unitree_modify_data(&motor->motorCmd_send);
    unitree_send_cmd(motor);
    unitree_receive_data(motor);
}

/**
 * 速度模式
 * @param motor_id
 * @param w     输出轴速度(弧度制)
 * @param kw    阻尼系数
 */
void unitree_set_speed(MotorData_t* motor, float w, float kw){
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
    unitree_modify_data(&motor->motorCmd_send);
    unitree_send_cmd(motor);
    unitree_receive_data(motor);
}

#endif
