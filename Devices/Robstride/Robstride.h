/**
  ******************************************************************************
  @file     Robstride.h
  @brief    RobStride电机CAN通信驱动： 
                - 电机初始化（配置CAN ID/通信参数）
                - 电机数据解析（角度/速度/扭矩/温度等状态读取）
                - 电机控制指令（使能/失能/零位设置/CAN ID修改）
                - 多模式运动控制（力矩/速度/位置/CSP/PP插补）
                - 参数读写（单个参数读取/写入/数据保存/波特率修改）
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-11-26 (Created) | 2026-02-19 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-19 [v1.0] Icol Boom: 创建初始版本
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_ROBOSTRIDE_H
#define DEVICE_ROBOSTRIDE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define Robostride_hcan hfdcan2

// 各种控制模式
#define move_control_mode   0 //运控模式
#define Pos_control_mode    1 //PP位置模式
#define Speed_control_mode  2 //速度模式
#define Elect_control_mode  3 //电流模式
#define Set_Zero_mode       4 //零点模式
#define CSP_control_mode    5 //CSP位置模式

// 通信地址
#define Communication_Type_Get_ID               0x00 //获取设备的ID和64位MCU唯一标识符`
#define Communication_Type_MotionControl        0x01 //运控模式用来向主机发送控制指令
#define Communication_Type_MotorRequest         0x02 //用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable          0x03 //电机使能运行
#define Communication_Type_MotorStop            0x04 //电机停止运行
#define Communication_Type_SetPosZero           0x06 //设置电机机械零位
#define Communication_Type_Can_ID               0x07 //更改当前电机CAN_ID
#define Communication_Type_GetSingleParameter   0x11 //读取单个参数
#define Communication_Type_SetSingleParameter   0x12 //设定单个参数
#define Communication_Type_ErrorFeedback        0x15 //故障反馈帧
//使用以下模式时请注意电机驱动版本号是否 >= 0.13.0
#define Communication_Type_MotorDataSave            0x16 //电机数据保存帧
#define Communication_Type_BaudRateChange           0x17 //电机波特率修改帧，重新上电生效
#define Communication_Type_ProactiveEscalationSet   0x18 //电机主动上报

/* Private type --------------------------------------------------------------*/
// 数据读写结构体
typedef struct {
    uint16_t index;
    float data;
} RobStride_data_read_write_one;

static const uint16_t RobStride_Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018,
                                                0x7019, 0x701A, 0x701B, 0x701C};

// 可读写的参数结构体
typedef struct {
    RobStride_data_read_write_one run_mode;           // 0:运控模式 1:位置模式 2:速度模式 3:电流模式 4:零点模式 uint8  1byte
    RobStride_data_read_write_one iq_ref;             // 电流模式Iq指令    float 4byte -11~11A
    RobStride_data_read_write_one spd_ref;            // 转速模式转速指令  float 4byte -50~50rad/s
    RobStride_data_read_write_one imit_torque;        // 转矩限制          float 4byte 0~6Nm
    RobStride_data_read_write_one cur_kp;             // 电流的 Kp         float 4byte 默认值 0.17
    RobStride_data_read_write_one cur_ki;             // 电流的 Ki         float 4byte 默认值 0.012
    RobStride_data_read_write_one cur_filt_gain;      // 电流滤波系数      float 4byte 0~1.0，默认值0.1
    RobStride_data_read_write_one loc_ref;            // 位置模式角度指令  float 4byte rad
    RobStride_data_read_write_one limit_spd;          // 位置模式速度设置  float 4byte 0~50rad/s
    RobStride_data_read_write_one limit_cur;          // 速度位置模式电流设置 float 4byte 0~11A
    // 以下只可读
    RobStride_data_read_write_one mechPos;            // 负载端计圈机械角度 float 4byte rad
    RobStride_data_read_write_one iqf;                // iq 滤波值         float 4byte -11~11A
    RobStride_data_read_write_one mechVel;            // 负载端转速        float 4byte -50~50rad/s
    RobStride_data_read_write_one VBUS;               // 母线电压          float 4byte V
} RobStride_data_read_write;

/* Exported macros -----------------------------------------------------------*/
extern RobStride_Motor EL05;

/* Exported types ------------------------------------------------------------*/
// 电机位置信息结构体
typedef struct {
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    int pattern; // 电机模式（0复位1标定2运行）
} RobStride_Motor_Info;

// 电机设置结构体
typedef struct {
    int set_motor_mode;     // 电机模式
    float set_current;      // 电流模式Iq指令
    float set_speed;        // 转速模式转速指令
    float set_acceleration; // 加速度设置
    float set_Torque;       // 转矩设置
    float set_angle;        // 角度设置
    float set_limit_cur;    // 电流限制
    float set_limit_speed;  // 速度限制
    float set_Kp;           // Kp设置
    float set_Ki;           // Ki设置
    float set_Kd;           // Kd设置
} RobStride_Motor_Set;

// 电机结构体
typedef struct {
    uint8_t CAN_ID;                  // CAN ID (默认127(0x7f) 可以通过上位机和通信类型1查看)
    uint64_t Unique_ID;              // 64位MCU唯一标识符
    uint16_t Master_CAN_ID;          // 主机ID （会在初始化函数中设定为0xFD）

    RobStride_Motor_Set Motor_Set_All;  // 设定值
    uint8_t error_code;

    RobStride_Motor_Info Pos_Info;      // 回传值
    RobStride_data_read_write drw;      // 电机数据
} RobStride_Motor;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void RobStride_Motor_Init(RobStride_Motor *motor, uint8_t CAN_Id);
//uint8_t mapFaults(uint16_t fault16);
void RobStride_Motor_Analysis(RobStride_Motor *motor, uint8_t *DataFrame, uint32_t ID_ExtId);

void RobStride_Get_CAN_ID(RobStride_Motor *motor);
void RobStride_Motor_move_control(RobStride_Motor *motor, float Torque, float Angle, float Speed, float Kp, float Kd);
void RobStride_Enable_Motor(RobStride_Motor *motor);
void RobStride_Disable_Motor(RobStride_Motor *motor, uint8_t clear_error);
void Set_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index, float Value);
void Get_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index);
void Set_CAN_ID(RobStride_Motor *motor, uint8_t Set_CAN_ID);
void Set_ZeroPos(RobStride_Motor *motor);
void RobStride_Motor_MotorDataSave(RobStride_Motor *motor);
void RobStride_Motor_BaudRateChange(RobStride_Motor *motor, uint8_t F_CMD);
void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor *motor, uint8_t F_CMD);

void data_read_write_init(RobStride_data_read_write *drw, const uint16_t *index_list);
void RobStride_Motor_Pos_control(RobStride_Motor *motor, float Angle, float Speed, float Acceleration);
void RobStride_Motor_CSP_control(RobStride_Motor *motor, float Angle, float limit_spd);
void RobStride_Motor_Speed_control(RobStride_Motor *motor, float Speed, float limit_cur, float Acceleration);
void RobStride_Motor_current_control(RobStride_Motor *motor, float current);
void RobStride_Motor_Set_Zero_control(RobStride_Motor *motor);
void RobStride_SetMode(RobStride_Motor *motor, uint8_t mode);

#endif //DEVICE_ROBOSTRIDE_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
