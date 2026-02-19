//
// Created by Icol_Lee on 2025/9/20.
//

#ifndef DEVICE_M3508_H
#define DEVICE_M3508_H

#include <stdint.h>

#define DJI_ID_1 0x201
#define DJI_ID_2 0x202
#define DJI_ID_3 0x203
#define DJI_ID_4 0x204
#define DJI_ID_5 0x205
#define DJI_ID_6 0x206
#define DJI_ID_7 0x207
#define DJI_ID_8 0x208

typedef struct{
    FDCAN_HandleTypeDef *can_handle;
    /*电调属性*/
    uint8_t     id;             // 电调id
    uint16_t    id_group;       // 电调接收端标识符

    /*反馈信息*/
    float       speed_rpm;      // 转速               RPM
    float       real_current;   // 实际转矩电流        [-20,20] A
    uint16_t 	pos;            // 转子机械角度        [0,8191]
    uint8_t     temp;           // 电机温度            ℃

    /*处理反馈*/
    uint8_t     offset_flag;    // 是否存储初始位置
    uint16_t	offset_pos;     // 初始转子机械角度      [0,8191]
    uint16_t 	last_pos;       // 上一次转子机械角度    [0,8191]
    int32_t		round_cnt;      // 转子计圈
    int32_t		all_pos;        // 实际转子机械角度（从上电位置开始计算）
    float       angle;          // 实际转子角度

    /*控制信息*/
    int16_t     given_current;  // 控制电流             [-16384, 16384]
    uint8_t     control_row;    // 控制指令对应的行号
    uint8_t     controlH_col;   // 控制指令高8位的列号
    uint8_t     controlL_col;   // 控制指令低8位的列号
}DJI_t;


// 实例化
extern DJI_t m3508_Switch;

void m3508_init(DJI_t *ptr, uint8_t id, FDCAN_HandleTypeDef *hfdcan);
void m3508_receive(DJI_t *ptr, const uint8_t rx[8]);
void m3508_send(DJI_t *ptr, int16_t iq);

#endif //DEVICE_M3508_H

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
