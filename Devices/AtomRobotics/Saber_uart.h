/**
  ******************************************************************************
  @file     Saber_uart.c
  @brief    Atom-Robotics Saber系列惯导模块串口协议驱动： 
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-02-09 (Created) | 2026-02-21 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-21 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#ifndef DEVICE_SABER_UART_H
#define DEVICE_SABER_UART_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
// Preamble
#define Saber_Preamble1 0x41
#define Saber_Preamble2 0x78

// CID
// Response from Device
#define RETURN_CODE_OK                      0000b
#define RETURN_CODE_ERROR_CLASS             0001b
#define RETURN_CODE_ERROR_CMD               0010b
#define RETURN_CODE_ERROR_BCC               0011b
#define RETURN_CODE_ERROR_FOOTER            0100b
#define RETURN_CODE_ERROR_PAYLOAD           0101b
#define RETURN_CODE_ERROR_OPERATION         0110b
#define RETURN_CODE_ERROR_UNSUPPORT         0111b
#define RETURN_CODE_ERROR_TIME_OUT          1000b
#define RETURN_CODE_ERROR_FLASH_OPERATION   1001b
#define RETURN_CODE_ERROR_INVALID_PARA      1010b
#define RETURN_CODE_FIRMWARE_UPDATE         1011b
#define RETURN_CODE_ERROR_STATE             1100b
#define RETURN_CODE_ERROR_UART_BAUDWIDTH    1101b
#define RETURN_CODE_ERROR_SENSOR_INIT       1111b

// Class ID
#define Saber_Operation                     0001b
#define Saber_Information                   0010b
#define Saber_Sensor                        0011b
#define Saber_Algorithm                     0100b
#define Saber_Communication                 0101b
#define Saber_HostConfigure                 0110b
#define Saber_FirmwareUpdate                1010b
#define Saber_Debug                         1110b

// PID
// SABER_TEMPERATURE_CLASS
#define Saber_Temperature                   0x8000  // 温度 float 单位：℃
// SABER_ACCELERATION_CLASS
#define Saber_Raw_Data_Acc                  0x8400  // 加速度计原始数据 short 单位：g
#define Saber_Cal_Acc                       0x8800  // 加速度计校准数据 float 单位：g
#define Saber_Kal_ACC                       0x8801  // 加速度计Kalman滤波数据 float 单位：g
// SABER_GYROSCOPE_CLASS
#define Saber_Raw_Data_Gyro                 0x8401  // 陀螺仪原始数据 short 单位：dps
#define Saber_Cal_Gyro                      0x8C00  // 陀螺仪校准数据 float 单位：dps
#define Saber_Kal_Gyro                      0x8C01  // 陀螺仪Kalman滤波数据 float 单位：dps
// SABER_MAGNETOMETER_CLASS
#define Saber_Raw_Data_Mag                  0x8402  // 磁力计原始数据 short 单位：mGauss
#define Saber_Cal_Mag                       0x9000  // 磁力计校准数据 float 单位：mGauss
#define Saber_Kal_Mag                       0x9001  // 磁力计Kalman数据 float 单位：mGauss
#define Saber_Mag_Dev                       0x9002  // 磁力计偏差数据 float （绝对值大于0.05，则认为磁场干扰较大） 单位：mGauss
// SABER_BAROMETER_CLASS
#define Saber_Raw_Data_Baro                 0x8403  // 气压计原始数据 int 单位：pa
#define Saber_Kal_Baro                      0x9401  // 气压计Kalman滤波数据 int 单位：pa
// SABER_GNSS_CLASS
#define Saber_Gnss_PVT_Data                 0xA000  // GNSS PVT原始数据
#define Saber_Gnss_Satellites_Data          0xA001  // 卫星数据
// SABER_ORIENTATION_CLASS
#define Saber_Quaternion                    0xB000  // 四元数数据（Q0-Q3） float
#define Saber_Euler                         0xB001  // 欧拉角 float 单位：°
#define Saber_Rotation_Matrix               0xB002  // 余弦矩阵 9个float数据
#define Saber_DualGNSSCompass               0xB003
// SABER_MOTION_CLASS
#define Saber_Linear_Acc                    0xB800  // 线性加速度 float 单位：m/s²
#define Saber_Velocity                      0xB802  // 速度 float 单位：m/s
#define Saber_Distance                      0xB403  // 位移 float 单位：m
#define Saber_Heave_Motion                  0xB803  // 升沉 float 单位：m
#define Saber_Altitude_Ellipsoid            0xB400  // 高度（海拔） float 单位：m
#define Saber_LonLat                        0xB402  // 经纬度 double 单位：°
#define Saber_Odometer_Data                 0xB500  // 轮速计 double 单位：m/s
// SABER_MISC_CLASS
#define Saber_Packet_Counter                0xC000  // 数据包编号 uint
#define Saber_OS_Time                       0xC002  // 操作系统参考时间 uint 单位：ms（B0-B3）    us（B4-B5）
#define Saber_DeltaT                        0xC006  // 采样间隔时间 float 单位：ms
#define Saber_Status_Word                   0xC400  // 状态字

// TAIL
#define Saber_Tail                          0x6D

/* Private type --------------------------------------------------------------*/
// Preamble
typedef struct {
    uint8_t Preamble1:8;    // 固定为0x41
    uint8_t Preamble2:8;    // 固定为0x78
} Saber_Preamble;

// CID
typedef struct {
    uint8_t RFD:4;          // Return from Device    4 Bit
    uint8_t CID:4;          // Class ID              4 Bit
} Saber_CID;

// Raw_Data_Euler
typedef struct {
    uint16_t PID:16;        // Packet ID         2 Byte
    uint8_t Length:8;       // 数据包长度(0x10)    1 Byte
    uint32_t Roll:32;       // 翻滚角             4 Byte
    uint32_t Pitch:32;      // 俯仰角             4 Byte
    uint32_t Yaw:32;        // 偏航角             4 Byte
} Saber_Euler_t;

// IMU通信结构体
typedef struct {
    Saber_Preamble Preamble;    // 2  Byte
    uint8_t MADDR:8;            // 1  Byte
    Saber_CID CID;              // 1  Byte
    uint8_t MID:8;              // 1  Byte
    uint8_t PL:8;               // 1  Byte
    Saber_Euler_t Euler;        // 15 Byte
    uint8_t BCC;                // 1  Byte
    uint8_t TAIL;               // 1  Byte
} Saber_Data;

// 欧拉角信息
typedef struct {
    float Roll;
    float Pitch;
    float Yaw;
}Euler_t;

/* Exported macros -----------------------------------------------------------*/
#define IMU_PREAMBLE_LENGTH         2
#define IMU_MADDR_LENGTH            1
#define IMU_CID_LENGTH              1
#define IMU_MID_LENGTH              1
#define IMU_PL_LENGTH               1
#define IMU_DATA_PACKAGE_LENGTH     19
#define IMU_BCC_LENGTH              1
#define IMU_TAIL_LENGTH             1
#define IMU_DATA_PL                 27

/* Exported types ------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
extern Euler_t Euler; // 用于储存姿态角信息

/* Exported function declarations ---------------------------------------------*/
void Saber_GetMessage(uint8_t* data);

#endif //DEVICE_SABER_UART_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
