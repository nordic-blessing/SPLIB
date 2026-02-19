//
// Created by Icol_Lee on 2025/6/22.
//

#ifndef DEVICE_UNITREE_MOTOROUPUT_H
#define DEVICE_UNITREE_MOTOROUPUT_H

#include "unitreeMotor.h"
#include "usart.h"

#define UNITREE_GO_UART    huart2
#define UNITREE_A1_UART    huart2

void Unitree_init(MotorData_t* motor, enum MotorType motorType, uint8_t id);
void Unitree_receive_data(MotorData_t* motor);
void Unitree_get_motor(MotorData_t* motor);
void Unitree_set_angle(MotorData_t* motor, float rad, float kp, float kw);
void Unitree_set_speed(MotorData_t* motor, float w, float kw);

#endif //DEVICE_UNITREE_MOTOROUPUT_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
