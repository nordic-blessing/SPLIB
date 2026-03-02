/**
  ******************************************************************************
  @file     DrEmpower_can.h
  @brief    大然一体化关节电机 CAN 驱动：
                - 位置模式
                - 速度模式
                - 力矩模式
                - 获取电机参数
            功能与官方驱动基本一致，根据队内使用情况对官方库做了简化，去除了较少使用的API
            以及部分空循环
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-03-15 (Created) | 2026-3-2 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-03-2 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ------------------------------------------------------------------------------
  @example
    具体使用与大然官方驱动一致
    - 回调函数参考
    void func_fdcan1_DrEmpower(CAN_RxBuffer* rxBuffer) {
        uint8_t CMD = rxBuffer->header.Identifier & 0x1F;
        if (CMD == 0x1E) {
            DrEmpower_Receive(rxBuffer->data);
        }
    }
    - 配置并启动CAN（详情参考bsp_can的使用说明）
    CAN_Filter_Mask_Config(&hfdcan1, CanFilter_0|CanFifo_0|Can_StdId|Can_DataType, 0x1E, 0x01F);
    CAN_Start_IT(&hfdcan1, CanFifo_0, func_fdcan1_DrEmpower);
  ------------------------------------------------------------------------------
  @attention
    - 使用前请在`splib_config.h`中使能`USE_SPLIB_CAN`或`USE_SPLIB_FDCAN`
    - 回调函数运行在中断上下文，禁止使用延时、打印、大量运算等阻塞性操作
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
    让后人知晓如何使用该驱动
    - 本驱动仅测试了STM32G4系列的部分型号，依赖STM32 HAL库底层初始化
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#include "splib.h"

#if USE_SPLIB_DREMPOWER

/* Includes ------------------------------------------------------------------*/
#include "DrEmpower_can.h"

/* Private define ------------------------------------------------------------*/
#define DrEmpower_hcan  hfdcan1

/* Private variables ---------------------------------------------------------*/
uint8_t DrEmpower_Receive_Flag = 0;
float DrEmpower_Receive_Temp;
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

// IEEE 754浮点数转十六进制
union float_hex{
    float floatValue;
    uint32_t hexValue;
};
union float_hex Dr_f_h;

/**
 * 设置电机角度
 * @param id    电机ID
 * @param mode  模式 1：跟随；2：梯形； 3：前馈力矩
 * @param angle 目标角度
 * @param speed 转速
 * @param param mode=1 ： 输入滤波带宽； mode = 2 ： 角加速度 ； mode = 2 ： 力矩
 */
void DrEmpower_Set_Angle(uint8_t id, float angle, uint16_t speed, uint16_t param, uint8_t mode) {
    uint8_t CAN_ID = id;
    uint8_t CMD_ID;
    uint8_t tx_data[8] = {0};

    switch (mode) {
        case 1: {
            CMD_ID = 0x19;
        }
            break;
        case 2: {
            CMD_ID = 0x1A;
        }
            break;
        case 3: {
            CMD_ID = 0x1B;
        }
            break;
        default:
            return;
    }

    speed *= 100;
    param *= 100;
    Dr_f_h.floatValue = angle;

    tx_data[0] = (Dr_f_h.hexValue) & 0xFF;
    tx_data[1] = (Dr_f_h.hexValue >> 8) & 0xFF;
    tx_data[2] = (Dr_f_h.hexValue >> 16) & 0xFF;
    tx_data[3] = (Dr_f_h.hexValue >> 24) & 0xFF;
    tx_data[4] = speed & 0xFF;
    tx_data[5] = (speed >> 8) & 0xFF;
    tx_data[6] = param & 0xFF;
    tx_data[7] = (param >> 8) & 0xFF;
    CAN_SendStdData(&DrEmpower_hcan, CAN_ID << 5 | CMD_ID, tx_data, 8);
}

/**
 * 设置电机速度
 * @param id    电机ID
 * @param speed 目标速度
 * @param param mode=0：前馈力矩； mode!=0：目标加转速度
 * @param mode  mode=0：直接控制： mode!=0：匀加速控制
 */
void DrEmpower_Set_Speed(uint8_t id, float speed, uint16_t param, uint16_t mode) {
    uint8_t CAN_ID = id;
    uint8_t CMD_ID = 0x1C;
    uint8_t tx_data[8] = {0};

    param *= 100;
    Dr_f_h.floatValue = speed;

    tx_data[0] = (Dr_f_h.hexValue) & 0xFF;
    tx_data[1] = (Dr_f_h.hexValue >> 8) & 0xFF;
    tx_data[2] = (Dr_f_h.hexValue >> 16) & 0xFF;
    tx_data[3] = (Dr_f_h.hexValue >> 24) & 0xFF;
    tx_data[4] = param & 0xFF;
    tx_data[5] = (param >> 8) & 0xFF;
    tx_data[6] = mode & 0xFF;
    tx_data[7] = (mode >> 8) & 0xFF;
    CAN_SendStdData(&DrEmpower_hcan, CAN_ID << 5 | CMD_ID, tx_data, 8);
}

/**
 * 设置电机力矩
 * @param id    电机ID
 * @param speed 目标力矩
 * @param param mode=0：参数无效； mode!=0：力矩增速
 * @param mode  mode=0：直接控制： mode!=0：力矩匀加控制
 */
void DrEmpower_Set_Torque(uint8_t id, float torque, uint16_t param, uint16_t mode) {
    uint8_t CAN_ID = id;
    uint8_t CMD_ID = 0x1D;
    uint8_t tx_data[8] = {0};

    param *= 100;
    Dr_f_h.floatValue = torque;

    tx_data[0] = (Dr_f_h.hexValue) & 0xFF;
    tx_data[1] = (Dr_f_h.hexValue >> 8) & 0xFF;
    tx_data[2] = (Dr_f_h.hexValue >> 16) & 0xFF;
    tx_data[3] = (Dr_f_h.hexValue >> 24) & 0xFF;
    tx_data[4] = param & 0xFF;
    tx_data[5] = (param >> 8) & 0xFF;
    tx_data[6] = mode & 0xFF;
    tx_data[7] = (mode >> 8) & 0xFF;
    CAN_SendStdData(&DrEmpower_hcan, CAN_ID << 5 | CMD_ID, tx_data, 8);
}

/**
 * 读取电机参数
 * @param id        电机ID
 * @param address   参数键码
 * @param data_type 数据类型
 */
void DrEmpower_Read_property(uint8_t id, uint16_t address, uint16_t data_type) {
    uint8_t CAN_ID = id;
    uint8_t CMD_ID = 0x1E;
    uint8_t tx_data[8] = {0};

    tx_data[0] = (address) & 0xFF;
    tx_data[1] = (address >> 8) & 0xFF;
    tx_data[2] = (data_type) & 0xFF;
    tx_data[3] = (data_type >> 8) & 0xFF;

    DrEmpower_Receive_Flag = 0;
    CAN_SendStdData(&DrEmpower_hcan, CAN_ID << 5 | CMD_ID, tx_data, 8);
    DELAY(1);
}

/**
 * 获取电机角度
 * @param id
 */
float DrEmpower_Get_Angle(uint8_t id){
    DrEmpower_Read_property(id, 38001, 0);
    if(DrEmpower_Receive_Flag){
        return DrEmpower_Receive_Temp;
    }
}

/**
 * 获取电机速度
 * @param id
 */
float DrEmpower_Get_Speed(uint8_t id){
    DrEmpower_Read_property(id, 38002, 0);
    if(DrEmpower_Receive_Flag){
        return DrEmpower_Receive_Temp;
    }
}

/**
 * 获取电机力矩
 * @param id
 */
float DrEmpower_Get_Torque(uint8_t id){
    DrEmpower_Read_property(id, 38003, 0);
    if(DrEmpower_Receive_Flag){
        return DrEmpower_Receive_Temp;
    }
}

/**
 * 获取电机温度
 * @param id
 */
float DrEmpower_Get_Temp(uint8_t id){
    DrEmpower_Read_property(id, 37002, 0);
    if(DrEmpower_Receive_Flag){
        return DrEmpower_Receive_Temp;
    }
}

/**
 * 解析电机反馈数据
 * @param handle
 * @param data
 */
void DrEmpower_Receive(const uint8_t* rxdata) {
    DrEmpower_Receive_Flag = 0;
    Dr_f_h.hexValue = rxdata[7] << 24 | rxdata[6] << 16 | rxdata[5] << 8 | rxdata[4];
    DrEmpower_Receive_Temp = Dr_f_h.floatValue;
    DrEmpower_Receive_Flag = 1;
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
