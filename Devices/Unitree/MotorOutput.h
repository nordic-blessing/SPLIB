//
// Created by Icol_Lee on 2025/6/22.
//

#ifndef DEVICE_UNITREE_MOTOROUPUT_H
#define DEVICE_UNITREE_MOTOROUPUT_H

#include "usart.h"
#include "unitreeMotor.h"

#define MOTOR_ID_NUM    10
#define UNITREE_UART    huart2

void unitree_init(MotorData_t* motor, enum MotorType motorType, uint8_t id);
void unitree_receive_data(MotorData_t* motor);
void unitree_get_motor(MotorData_t* motor);
void unitree_set_angle(MotorData_t* motor, float rad, float kp, float kw);
void unitree_set_speed(MotorData_t* motor, float w, float kw);

#endif //DEVICE_UNITREE_MOTOROUPUT_H
