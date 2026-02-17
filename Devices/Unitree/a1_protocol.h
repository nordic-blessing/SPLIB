//
// Created by Icol_Lee on 2025/11/7.
//
#ifndef DEVICE_UNITREE_A1_PROTOCOL_H
#define DEVICE_UNITREE_A1_PROTOCOL_H

// 发送用单个数据数据结构
#pragma pack(1)

typedef union{
        int32_t           L;
        uint8_t       u8[4];
       uint16_t      u16[2];
       uint32_t         u32;
          float           F;
}COMData32;

#pragma pack()


#pragma pack(1)

typedef struct {
    uint8_t start[2];   // 包头，固定为0xFE,0xEE
    uint8_t motorID;    // 电机标号， 可以为0、1、2、0xBB，0xBB代表向所有电机广播
    uint8_t reserved;   // 预留位，可忽略
}COMHead;

#pragma pack()


// 电机控制命令数据包
#pragma pack(1)

typedef struct {
    uint8_t mode;       // 电机运行模式，可为0（停转）、5（开环缓慢转动）、10（闭环伺服控制）
    uint8_t ModifyBit;  // 电机内部控制参数修改位，可忽略
    uint8_t ReadBit;    // 电机内部控制参数发送位，可忽略
    uint8_t reserved;   // 预留位，可忽略

    COMData32 Modify;   // 电机参数修改数据，可忽略

    //实际给FOC的指令力矩为：
    //K_P*delta_Pos + K_W*delta_W + T
    int16_t T;          // 期望关节的前馈力矩 × 256
    int16_t W;          // 期望关节速度 （电机本身的速度） × 128
    int32_t Pos;        // 期望关节位置 × 16384/6.2832

    int16_t K_P;        // 关节刚度系数 × 2048
    int16_t K_W;        // 关节速度系数 × 1024

    uint8_t LowHzMotorCmdIndex;     // 电机低频率控制命令的索引, 可忽略
    uint8_t LowHzMotorCmdByte;      // 电机低频率控制命令，可忽略
	
    COMData32  Res[1];  // 预留位，可忽略

}MasterComdV3;

typedef struct {
    COMHead head;    
    MasterComdV3 Mdata;
    COMData32 CRCdata;
}MasterComdDataV3;  // 控制数据 (34 Byte)

#pragma pack()


// 电机反馈命令数据包
#pragma pack(1)

typedef struct {
    uint8_t mode;           // 电机当前的运行模式
    uint8_t ReadBit;        // 表示内部电机控制参数修改是否成功，可忽略
    int8_t  Temp;           // 电机当前平均温度   
    uint8_t MError;         // 电机报错信息
    COMData32 Read;         // 读取电机内部控制参数，可忽略

    int16_t     T;          // 当前实际电机输出力矩， × 256倍描述
    int16_t     W;          // 当前实际电机转速， × 128倍描述
    float      LW;          // 也表示电机实际转速，但是已经过滤波，会有延迟，不推荐使用
    int16_t     W2;         // 位关节编码器预留，可忽略
    float       LW2;         // 位关节编码器预留，可忽略
    int16_t     Acc;         // 当前电机转动加速度， × 1 倍描述
    int16_t     OutAcc;      // 为关节编码器预留，可忽略
    int32_t     Pos;          // 当前电机角度位置， × 16384/6.2832 倍描述
    int32_t     Pos2;         // 为关节编码器预留，可忽略

    int16_t     gyro[3];    // 电机驱动板6轴传感器数据
    int16_t     acc[3];
    int16_t     Fgyro[3];   // 足端IMU预留，可忽略
    int16_t     Facc[3];
    int16_t     Fmag[3];
    uint8_t     Ftemp;      // 足端传感器温度，可忽略
    int16_t     Force16;    // 足端力传感器高16位数据，可忽略
    int8_t      Force8;     // 足端力传感器低8位数据，可忽略
    uint8_t     FError;     // 足端传感器错误标识，可忽略
	
    int8_t      Res[1];     // 预留位，可忽略
	
}ServoComdV3;

typedef struct {
    COMHead     head;
    ServoComdV3 Mdata;
    COMData32   CRCdata;
}ServoComdDataV3;   // 反馈数据 （78 Byte）

#pragma pack()

#endif //DEVICE_UNITREE_A1_PROTOCOL_H