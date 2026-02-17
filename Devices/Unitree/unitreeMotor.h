#ifndef DEVICE_UNITREE_MOTOR_H
#define DEVICE_UNITREE_MOTOR_H

#include <string.h>
#include "a1_protocol.h"
#include "gom_protocol.h"
#include "crc_ccitt.h"

enum MotorType{
    GO_M8010_6, // 4M bps
    A1,         // 4.8M bps
    B1,         // 6.5M bps
};

/// @brief 电机指令结构体
typedef struct {
    enum MotorType motorType;   // 电机类型
    unsigned short id;          // 电机ID
    unsigned short mode;        // 电机模式
    float T;                    // 期望关节的输出力矩(Nm)
    float W;                    // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;                  // 期望关节位置(rad)
    float K_P;                  // 关节刚度系数
    float K_W;                  // 关节速度系数

    RIS_ControlData_t GO_M8010_6_motor_send_data;   // 电机发送数据结构体
    MasterComdDataV3 A1B1_motor_send_data;
} MotorCmd_t;

/// @brief 电机反馈结构体
typedef struct {
    enum MotorType motorType;

    unsigned char motor_id; // 电机ID
    unsigned char mode;     // 电机模式
    int Temp;               // 温度
    int MError;             // 错误码
    float T;                // 当前实际电机输出力矩(Nm)
    float W;                // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;              // 当前电机位置(rad)
    float Acc;              // 当前电机的转动加速度(A1电机反馈)
    int correct;            // 接收数据是否完整(1完整，0不完整)
    uint16_t calc_crc;

    MotorCmd_t motorCmd_send;

    RIS_MotorData_t GO_M8010_6_motor_recv_data; // 电机接收数据结构体
    ServoComdDataV3 A1B1_motor_recv_data;
} MotorData_t;

void unitree_modify_data(MotorCmd_t* motor_s);
uint8_t unitree_extract_data(MotorData_t* motor_r);


#endif //DEVICE_UNITREE_MOTOR_H

