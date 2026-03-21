/**
  ******************************************************************************
  @file     mA_Acquisition.c
  @brief    4-20mA电流采集转RS485数字输出的Modbus协议驱动：
  @author   Icol Boom < icolboom4@gmail.com >
  @date     2026-03-04 (Created) | 2026-03-04 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-03-04 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ------------------------------------------------------------------------------
  @example
    - 使用示例
        Acquisition_t DT35_FM;

        DT35_FM.id = 0x01;
        TickType_t xLastWakeTick;
        xLastWakeTick = osKernelGetTickCount();

        while(1) {
            Acquisition_GetValue(&DT35_FM);
            uart_printf("value:%d,%f\r\n", DT35_FM.value, (float) (DT35_FM.value - 4000) / 16000.0f * (470 - 10) + 10.0f);
            xLastWakeTick += pdMS_TO_TICKS(6);
            osDelayUntil(xLastWakeTick);
        }
    - 数字量转距离公式
        实际距离 = （反馈数字量 - 数字量下限） / 数字量幅度 * （最大测量距离 - 最小测量距离） + 最小测量距离
  ------------------------------------------------------------------------------
  @attention
    - 使用前需要先声明 Acquisition_t 实例，并为其中 id 元素赋值，同时在Acquisition_GetMessage配置解码
    - 模块极限反馈间隔为 6ms，请勿将数据的获取命令放于小于 6ms 的循环中！
    - 使用DT35时需要注意：当距离超过设置的上下限时，电流信号与距离不再成线性关系，本驱动仅负责数据的采集，如有需要可自行设置处理逻辑
    - 当前版本将Modbus协议规范当做特殊的私有协议进行处理，后续会补全Modbus的板级支持包
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
    让后人知晓如何使用该驱动
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#include "splib.h"

#if USE_SPLIB_ACQUISITION

/* Includes ------------------------------------------------------------------*/
#include "mA_Acquisition.h"

/* Private function declarations ---------------------------------------------*/
void Acquisition_GetMessage(uint8_t* data);
uint16_t Acquisition_Decode(uint8_t* data);

/* Private define ------------------------------------------------------------*/
#define ACQUISITION_UART    huart2
/* Private variables ---------------------------------------------------------*/
ProtocolHandler acquisition_uart_FM= {
        .package_length = ACQUISITION_RECEIVE_PL,
        .header = 0x01 << 8 | ACQUISITION_FUNC,
        .header_length = ACQUISITION_HEADER_LENGTH,
        .tail_flag = false,
        .callback = Acquisition_GetMessage,

        .buffer_index = 0,
        .header_found = false,
        .buffer = {0}
};
/* Private type --------------------------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 分配解码后的数据
 * @param data
 */
void Acquisition_GetMessage(uint8_t* data) {
    switch (data[0]) {
        case 0x01:
            DT35_T.value = Acquisition_Decode(data);
            break;
        default:
            break;
    }
}

/**
 * CRC校验
 * @param CRC_Ptr
 * @param LEN
 * @return
 */
uint16_t CRC_Check(uint8_t *CRC_Ptr, uint8_t LEN) {
    uint16_t CRC_Value = 0xFFFF;
    uint8_t i, j;

    for (i = 0; i < LEN; i++) {
        CRC_Value ^= *(CRC_Ptr + i);
        for (j = 0; j < 8; j++) {
            if (CRC_Value & 0x0001)
                CRC_Value = (CRC_Value >> 1) ^ 0xA001;
            else
                CRC_Value = (CRC_Value >> 1);
        }
    }
    CRC_Value = (CRC_Value >> 8) | (CRC_Value << 8); // 交换高低字节

    return CRC_Value;
}

/**
 * 发送数据获取命令
 * @param pAcquisition
 */
void Acquisition_GetValue(Acquisition_t* pAcquisition) {
    uint8_t data[8] = {0};
    data[0] = pAcquisition->id;
    data[1] = ACQUISITION_FUNC;
    data[5] = 0x01;
    uint16_t crc = CRC_Check(data, 6);
    data[6] = (crc >> 8) & 0xFF;
    data[7] = crc & 0xFF;

    HAL_UART_Transmit(&ACQUISITION_UART, data, sizeof(data) / sizeof(data[0]), 10);
}

/**
 * 解码反馈数据
 * @param data
 * @return
 */
uint16_t Acquisition_Decode(uint8_t* data) {
    if (CRC_Check(data, 5) == (data[5] << 8 | data[6]))
        return data[3] << 8 | data[4];
    else
        return 0;
}

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/