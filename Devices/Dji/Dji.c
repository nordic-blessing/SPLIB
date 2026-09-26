/**
******************************************************************************
  @file     Dji.c
  @brief    大疆电机CAN驱动
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2024-09-20 (Created) | 2026-09-26 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2024-09-20 [v1.0] Icol Boom: 创建初始版本，完成初步测试
    - 2026-09-26 [v1.0] Icol Boom: 补全注释
  ------------------------------------------------------------------------------
  @example
    - 使用示例
        // 声明变量
        DJI_t m3508_F;
        PID_t posF;
        PID_t velF;

        // 初始化CAN
        CAN_Filter_Mask_Config(&hfdcan1, CanFilter_0|CanFifo_0|Can_StdId|Can_DataType, 0x01E, 0x01F);
        CAN_Start_IT(&hfdcan1, CanFifo_0, func_fdcan1_Dji_C620);

        // 初始化电机控制参数
        Dji_init(&m3508_F, DJI_ID_1, &hfdcan2);
        initPID(&posF, 7000.0f, 7000.0f/3, 0.5f);
        setPIDParam(&posF, 70.0f, 0.0f, 800.0f);

        // 配置中断函数
        void func_fdcan1_Dji_C620(CAN_RxBuffer* rxBuffer) {
            m3508_receive(&m3508_F, rxBuffer->data);
        }
        （在初始化代码中调用如上代码）
  ------------------------------------------------------------------------------
  @attention
    - 驱动依赖于`bsp_can.c/h`，请务必在`splib_config.h`中使能`USE_SPLIB_CAN`或
        `USE_SPLIB_FDCAN`
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
    让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/

#include "splib.h"

#if USE_SPLIB_DJI

/* Includes ------------------------------------------------------------------*/
#include "Dji.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Dji_data[2][8] = {0};

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 电机结构体初始化
 * @param ptr
 * @param id
 * @param hfdcan
 */
void Dji_init(DJI_t *ptr, uint16_t id, FDCAN_HandleTypeDef *hfdcan) {
    ptr->can_handle = hfdcan;

    ptr->id = id;
    ptr->id_group = id > 0x204 ? 1 : 0;

    ptr->pos = 0;
    ptr->speed_rpm = 0;
    ptr->real_current = 0.0f;
    ptr->temp = 0;

    ptr->offset_flag = 0;
    ptr->offset_pos = 0;
    ptr->last_pos = 0;
    ptr->round_cnt = 0;
    ptr->all_pos = 0;
    ptr->angle = 0.0f;

    ptr->control_row = ptr->id_group;
    ptr->controlH_col = 2 * ((ptr->id + ptr->id_group) % 5) - 2;
    ptr->controlL_col = 2 * ((ptr->id + ptr->id_group) % 5) - 1;
}

/**
 * 接收电调反馈信息，此函数应该放置在接收中断中
 * @param ptr
 * @param rx    从中断中接受到的数据
 */
void m3508_receive(DJI_t *ptr, const uint8_t rx[]) {
    ptr->last_pos = ptr->pos;
    ptr->pos = (uint16_t) (rx[0] << 8 | rx[1]);
    ptr->speed_rpm = (int16_t) (rx[2] << 8 | rx[3]) / 19.0f;
    ptr->real_current = (float) (rx[4] << 8 | rx[5]) * 5.f / 16384.f;
    ptr->temp = rx[6];

    if ((float) (ptr->pos - ptr->last_pos) > 4096.0f)
        ptr->round_cnt--;
    else if ((float) (ptr->pos - ptr->last_pos) < -4096.0f)
        ptr->round_cnt++;

    //记录初始位置
    if (!ptr->offset_flag) {
        ptr->offset_pos = ptr->pos;
        ptr->round_cnt = 0;
        ptr->offset_flag = 1;
    }

    ptr->all_pos = ptr->round_cnt * 8192 + ptr->pos - ptr->offset_pos;
    ptr->angle = ((float) ptr->all_pos / 8192.0f) * 360.0f;
}

void m2006_receive(DJI_t *ptr, const uint8_t rx[]) {
    ptr->last_pos = ptr->pos;
    ptr->pos = (uint16_t) (rx[0] << 8 | rx[1]);
    ptr->speed_rpm = (int16_t) (rx[2] << 8 | rx[3]) / 36.0f;
    ptr->real_current = (float) (rx[4] << 8 | rx[5]) * 10.f / 10000.f;

    if ((float) (ptr->pos - ptr->last_pos) > 4096.0f)
        ptr->round_cnt--;
    else if ((float) (ptr->pos - ptr->last_pos) < -4096.0f)
        ptr->round_cnt++;

    //记录初始位置
    if (!ptr->offset_flag) {
        ptr->offset_pos = ptr->pos;
        ptr->round_cnt = 0;
        ptr->offset_flag = 1;
    }

    ptr->all_pos = ptr->round_cnt * 8192 + ptr->pos - ptr->offset_pos;
    ptr->angle = ((float) ptr->all_pos / 8192.0f) * 360.0f;
}

/**
 * 发送控制信息
 * @param ptr
 * @param iq 控制电流值
 */
#define ABS(x)  (x > 0 ? x : (- x))
void Dji_send(DJI_t *ptr, int16_t iq) {
    //控制电流范围 [-16384, 16384]
    if (ABS(iq) < 10000) {
        ptr->given_current = iq;
        Dji_data[ptr->control_row][ptr->controlH_col] = iq >> 8;
        Dji_data[ptr->control_row][ptr->controlL_col] = iq & 0xFF;
        CAN_SendStdData(ptr->can_handle, ptr->id_group ? 0x1FF : 0x200, Dji_data[ptr->control_row],
                        FDCAN_DLC_BYTES_8);
    }
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
