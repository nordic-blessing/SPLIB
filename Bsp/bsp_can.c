/**
  ******************************************************************************
  @file     bsp_can.c
  @brief    STM32(HAL) CAN/FDCAN驱动： 
                - 滤波器掩码配置
                - 中断回调注册
                - 标准/扩展帧发送
  @author   Icol Boom <icolboom4@gmail.com>
  @date     2025-09-27 (Created) | 2026-02-19 (Last modified)
  @version  v1.0
  ------------------------------------------------------------------------------
  CHANGE LOG :
    - 2026-02-19 [v1.0] Icol Boom: 创建初始版本，完成初步测试
  ------------------------------------------------------------------------------
  @example
    - 开启CAN : 调用`CAN_Start_IT()`，开启CAN，使能中断，并配置CAN接收处理函数
        `CAN_Start_IT(&hcan1, CanFIFO_0，CAN1_RxCallback)` // 注册接收回调至FIFO0，
        并开启CAN外设及中断
    - 滤波配置 : 调用`CAN_Filter_Mask_Config`进行滤波器配置（需注意配置格式）
        `CAN_Filter_Mask_Config(&hcan1, CanFilter_0|CanFIFO_0|Can_StdId|Can_DataType, 0x123, 0x3FF)` // 过滤器0，精准匹配标准ID 0x123的数据帧，分流至FIFO0
    - 数据发送 : 调用`CAN_SendStdData()`发送标准帧/`CAN_SendExtData()`发送扩展帧
        `CAN_SendStdData(&hcan1, 0x123, data, 2)` // 标准帧ID 0x123 发送2字节的data数据
        `CAN_SendExtData(...)`
  ------------------------------------------------------------------------------
  @attention
    - 使用前请在`splib_config.h`中使能`USE_SPLIB_CAN`或`USE_SPLIB_FDCAN`
    - 回调函数运行在中断上下文，禁止使用延时、打印、大量运算等阻塞性操作
    - 修改代码后需同步更新版本号、最后修改日期及CHANGE LOG，请务必保证注释清晰明确地
    让后人知晓如何使用该驱动
    - 本驱动仅测试了STM32F4/G4系列的部分型号，依赖STM32 HAL库底层初始化
  ******************************************************************************
  Copyright (c) 2026 ~ -, Sichuan University Pangolin Robot Lab.
  All rights reserved.
  ******************************************************************************
*/
#include "splib.h"

#if USE_SPLIB_CAN | USE_SPLIB_FDCAN

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static void (*pCAN1_FIFO0RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN2_FIFO0RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN3_FIFO0RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN1_FIFO1RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN2_FIFO1RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN3_FIFO1RxCpltCallback)(CAN_RxBuffer *);

/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
 * 开启CAN中断，设置回调函数
 * @param hcan  can句柄
 * @param pFunc 回调函数
 */
void CAN_Start_IT(CAN_HandleTypeDef* hcan, uint8_t FIFOx, void (*pFunc)(CAN_RxBuffer*)) {
#if USE_SPLIB_FDCAN
    /* 配置回调函数 */
    if(FIFOx == CanFifo_0) {
        if (hcan->Instance == FDCAN1) {
            pCAN1_FIFO0RxCpltCallback = pFunc;
        } else if (hcan->Instance == FDCAN2) {
            pCAN2_FIFO0RxCpltCallback = pFunc;
        } else if (hcan->Instance == FDCAN3) {
            pCAN3_FIFO0RxCpltCallback = pFunc;
        }
    }else if(FIFOx == CanFifo_1){
        if (hcan->Instance == FDCAN1) {
            pCAN1_FIFO1RxCpltCallback = pFunc;
        } else if (hcan->Instance == FDCAN2) {
            pCAN2_FIFO1RxCpltCallback = pFunc;
        } else if (hcan->Instance == FDCAN3) {
            pCAN3_FIFO1RxCpltCallback = pFunc;
        }
    }

    /* 开启FDCAN外设 */
    HAL_FDCAN_Start(hcan);// !FDCAN外设的开启需要在中断的使能之前

    /* 使能中断 */
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
#endif

#if USE_SPLIB_CAN
    /* 配置回调函数 */
    if(FIFOx == CanFifo_0) {
        if (hcan->Instance == CAN1) {
            pCAN1_FIFO0RxCpltCallback = pFunc;
        } else if (hcan->Instance == CAN2) {
            pCAN2_FIFO0RxCpltCallback = pFunc;
        }
    }else if(FIFOx == CanFifo_1){
        if (hcan->Instance == CAN1) {
            pCAN1_FIFO1RxCpltCallback = pFunc;
        } else if (hcan->Instance == CAN2) {
            pCAN2_FIFO1RxCpltCallback = pFunc;
        }
    }

    /* 开启CAN外设 */
    HAL_CAN_Start(hcan);// !CAN外设的开启需要在中断的使能之前

    /* 使能中断 */
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif
}

/**
 * 配置CAN过滤器掩码模式
 * @param hcan
 * @param para  [|Filter编号|FIFOx(1bit)|ID类型(1bit)|帧类型(1bit)|]
 * @param Id    [ID]
 * @param Mask  [掩码(0x3ff,0x1FFFFFFF)]
 * @brief   CAN模式下需要配置过滤器过滤帧的帧类型
 *          FDCAN模式下无需关注帧类型，只过滤ID
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef* hcan, uint8_t para, uint32_t Id, uint32_t Mask) {
#if USE_SPLIB_FDCAN
    FDCAN_FilterTypeDef canFliter = {0};

    canFliter.IdType = ((para >> 1) & 0x01) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;    // ID类型
    canFliter.FilterIndex = para >> 3;                                                  // 过滤器索引
    canFliter.FilterType = FDCAN_FILTER_MASK;                                           // 过滤器类型
    canFliter.FilterConfig = (para >> 2) & 0x01;                                        // 过滤器关联FIFO
    canFliter.FilterID1 = Id;
    canFliter.FilterID2 = Mask;                                                         // FDCAN模式下直接填入ID掩码即可

    /*FDCAN初始化*/
    HAL_FDCAN_ConfigFilter(hcan, &canFliter);
#endif

#if USE_SPLIB_CAN
    CAN_FilterTypeDef canFliter = {0};

    if ((para >> 1 & 0x01)) {
        /* ExtId */
        canFliter.FilterIdHigh = (Id << 3);
        canFliter.FilterIdLow = (Id << 3) | (((para >> 1) & 0x01) << 2) | ((para & 0x01) << 1);
        canFliter.FilterMaskIdHigh = (Mask << 3);
        canFliter.FilterMaskIdLow = (Mask << 3) | (1 << 2) | (1 << 1);
    } else {
        /* StdId */
        canFliter.FilterIdHigh = (Id << 5);
        canFliter.FilterIdLow = (((para >> 1) & 0x01) << 2) | ((para & 0x01) << 1);
        canFliter.FilterMaskIdHigh = (Mask << 5);
        canFliter.FilterMaskIdLow = (1 << 2) | (1 << 1);
    }
    canFliter.FilterFIFOAssignment = (para >> 2) ? CAN_FILTER_FIFO1 : CAN_FILTER_FIFO0;
    canFliter.FilterBank = para >> 3;
    canFliter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFliter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFliter.FilterActivation = ENABLE;
    canFliter.SlaveStartFilterBank = 14;

    /*CAN初始化*/
    HAL_CAN_ConfigFilter(hcan, &canFliter);
#endif
}

/**
 * CAN发送标准数据帧
 * @param hcan
 * @param StdId
 * @param pData
 * @param Len
 */
void CAN_SendStdData(CAN_HandleTypeDef* hcan, uint16_t StdId, uint8_t* pData, uint8_t Len) {
#if USE_SPLIB_FDCAN
    FDCAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.Identifier = StdId;
    CAN_TxHeader.DataLength = Len;
    CAN_TxHeader.IdType = FDCAN_STANDARD_ID;
    CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    CAN_TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(hcan, &CAN_TxHeader, pData);
    }
#endif

#if USE_SPLIB_CAN
    CAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.StdId = StdId;
    CAN_TxHeader.ExtId = 0;
    CAN_TxHeader.IDE = CAN_ID_STD;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = Len;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, pData, NULL);
    }
#endif
}

/**
 * CAN发送扩展数据帧
 * @param hcan
 * @param ExtId
 * @param pData
 * @param Len
 */
void CAN_SendExtData(CAN_HandleTypeDef* hcan, uint16_t ExtId, uint8_t* pData, uint8_t Len) {
#if USE_SPLIB_FDCAN
    FDCAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.Identifier = ExtId;
    CAN_TxHeader.DataLength = Len;
    CAN_TxHeader.IdType = FDCAN_EXTENDED_ID;
    CAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    CAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    CAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    CAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    CAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    CAN_TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) > 0) {
        HAL_FDCAN_AddMessageToTxFifoQ(hcan, &CAN_TxHeader, pData);
    }
#endif

#if USE_SPLIB_CAN
    CAN_TxHeaderTypeDef CAN_TxHeader = {0};

    CAN_TxHeader.StdId = 0;
    CAN_TxHeader.ExtId = ExtId;
    CAN_TxHeader.IDE = CAN_ID_EXT;
    CAN_TxHeader.RTR = CAN_RTR_DATA;
    CAN_TxHeader.DLC = Len;
    CAN_TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
        HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, pData, NULL);
    }
#endif
}

/**
 * @brief 重定义RxFifo0的中断回调函数
 */
#if USE_SPLIB_FDCAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (hfdcan->Instance == FDCAN1) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN1_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
                if (pCAN1_FIFO0RxCpltCallback != NULL) {
                    pCAN1_FIFO0RxCpltCallback(&CAN1_RxBuffer);
                }
            } else {}
        } else if (hfdcan->Instance == FDCAN2) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN2_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
                if (pCAN2_FIFO0RxCpltCallback != NULL) {
                    pCAN2_FIFO0RxCpltCallback(&CAN2_RxBuffer);
                }
            } else {}
        } else if (hfdcan->Instance == FDCAN3) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN3_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN3_RxBuffer.header, CAN3_RxBuffer.data) == HAL_OK) {
                if (pCAN3_FIFO0RxCpltCallback != NULL) {
                    pCAN3_FIFO0RxCpltCallback(&CAN3_RxBuffer);
                }
            } else {}
        }
    }
}
#endif

#if USE_SPLIB_CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN1_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
            if(pCAN1_FIFO0RxCpltCallback != NULL) {
                pCAN1_FIFO0RxCpltCallback(&CAN1_RxBuffer);
            }
        } else {}
    } else if (hcan->Instance == CAN2) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN2_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
            if(pCAN2_FIFO0RxCpltCallback != NULL) {
                pCAN2_FIFO0RxCpltCallback(&CAN2_RxBuffer);
            }
        } else {}
    }
}
#endif

/**
 * @brief 重定义RxFifo1的中断回调函数
 */
#if USE_SPLIB_FDCAN
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        /* Switch to user call back function. */
        if (hfdcan->Instance == FDCAN1) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN1_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
                if(pCAN1_FIFO1RxCpltCallback != NULL) {
                    pCAN1_FIFO1RxCpltCallback(&CAN1_RxBuffer);
                }
            } else {}
        } else if (hfdcan->Instance == FDCAN2) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN2_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
                if(pCAN2_FIFO1RxCpltCallback != NULL) {
                    pCAN2_FIFO1RxCpltCallback(&CAN2_RxBuffer);
                }
            } else {}
        } else if (hfdcan->Instance == FDCAN3) {
            /*!< CAN receive buffer */
            CAN_RxBuffer CAN3_RxBuffer;

            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN3_RxBuffer.header, CAN3_RxBuffer.data) == HAL_OK) {
                if(pCAN3_FIFO1RxCpltCallback != NULL) {
                    pCAN3_FIFO1RxCpltCallback(&CAN3_RxBuffer);
                }
            } else {}
        }
    }
}
#endif

#if USE_SPLIB_CAN
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN1_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN1_RxBuffer.header, CAN1_RxBuffer.data) == HAL_OK) {
            if(pCAN1_FIFO1RxCpltCallback != NULL) {
                pCAN1_FIFO1RxCpltCallback(&CAN1_RxBuffer);
            }
        } else {}
    } else if (hcan->Instance == CAN2) {
        /*!< CAN receive buffer */
        CAN_RxBuffer CAN2_RxBuffer;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2_RxBuffer.header, CAN2_RxBuffer.data) == HAL_OK) {
            if(pCAN2_FIFO1RxCpltCallback != NULL) {
                pCAN2_FIFO1RxCpltCallback(&CAN2_RxBuffer);
            }
        } else {}
    }
}
#endif

#endif

/************************ COPYRIGHT(C) Pangolin Robot Lab **************************/
